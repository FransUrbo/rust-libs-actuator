#![no_std]

use defmt::{debug, error, info, trace, Format};

use embassy_rp::adc::{Adc, AdcPin, Async, Channel, Config, InterruptHandler};
use embassy_rp::gpio::{AnyPin, Drive, Level, Output, Pull, SlewRate};
use embassy_rp::interrupt::typelevel::{Binding, ADC_IRQ_FIFO};
use embassy_rp::peripherals::ADC;
use embassy_rp::Peri;
use embassy_time::Timer;

//use core::marker::PhantomData;
use core::mem::transmute;

// ==============================
// Setup static values, measured on the actuator.

// The resistance on the actuator at MIN and MAX throw.
// Measured on the actuator at endpoints, using the `read-actuator-pot` application.
// The multimeter have different values though, so not sure exactly what value this is!
pub static RESISTANCE_THROW_MIN: u16 = 240; // Ω	-- Fully retracted
pub static RESISTANCE_THROW_MAX: u16 = 1950; // Ω	-- Fully extended

// Measured values:
// 'P' starts `RESISTANCE_FIRST_GEAR` (~12mm) from outer endpoint.
//   * 'P': 1630Ω
//   * 'R': 1230Ω
//   * 'N':  830Ω
//   * 'D':  430Ω
static RESISTANCE_FIRST_GEAR: u16 = 430;
static RESISTANCE_PER_GEAR: u16 = 400;

// The distance between 'P' and 'D' is 70mm.
static DISTANCE_BETWEEN_ENDPOINTS: u16 = 70;
static DISTANCE_BETWEEN_POSITIONS: f32 = (DISTANCE_BETWEEN_ENDPOINTS / 3) as f32; // mm

// The allowed +/- value between MIN and MAX gear position is +/- 4mm.
static DISTANCE_ALLOWED_GEAR_DIFFERENCE: u8 = 4; // mm

// -----

// NOTE: It is vital to get these two as exact as possible, because the
//       time per mm is solely based on this value! If we can't
//       correctly calculate how long it takes to move one millimeter,
//       then we'll never be able to move to exact gear positions.

// Moving the actuator between endpoints takes about 2.1s@12.0V.
// Make it public, so we can use it to test the actuator.
pub static TIME_THROW_TOTAL: u64 = 2100; // ms

// The total throw from MIN to MAX is `101.56mm` (`3.9984"`).
static DISTANCE_THROW_TOTAL: f32 = 101.56; // mm

// ==============================
// Calculate values based on the static's..

// These two are public, because if/when we need to test the actuator, we need to
// know them..
pub static TIME_THROW_1MM: f32 = (TIME_THROW_TOTAL as f32 / DISTANCE_THROW_TOTAL) as f32;
pub static TIME_THROW_GEAR: f32 = (TIME_THROW_1MM * DISTANCE_BETWEEN_POSITIONS) as f32;

pub static RESISTANCE_THROW_TOTAL: u16 = RESISTANCE_THROW_MAX - RESISTANCE_THROW_MIN;
pub static RESISTANCE_THROW_1MM: u16 =
    (RESISTANCE_THROW_TOTAL as f32 / DISTANCE_THROW_TOTAL) as u16;

static RESISTANCE_ALLOWED_GEAR_DIFFERENCE: u16 =
    (DISTANCE_ALLOWED_GEAR_DIFFERENCE as u16) * RESISTANCE_THROW_1MM;

// -----

// Pre-calculate the proper gear mode positions, based on the start position (Ω).
// (which is where the 'D' mode is - max pulled in).
static GEAR_D: u16 = RESISTANCE_FIRST_GEAR; // Button::D
static GEAR_N: u16 = GEAR_D + RESISTANCE_PER_GEAR; // Button::N
static GEAR_R: u16 = GEAR_N + RESISTANCE_PER_GEAR; // Button::R
static GEAR_P: u16 = GEAR_R + RESISTANCE_PER_GEAR; // Button::P

// How many times to read the actuator potentiometer and get an average of the reads.
// NOTE: 50ms delay between reads * this = time!! Don't query to many times, or we'll
//       have to wait forever!!
static READ_ACTUATOR_TIMES: u16 = 5;

// ==============================

#[derive(Copy, Clone, Format, PartialEq)]
#[repr(u8)]
pub enum Direction {
    Backward, // Retract
    Forward,  // Extend
}

#[derive(Copy, Clone, Format)]
#[repr(u8)]
pub enum GearModes {
    P,
    R,
    N,
    D,
}

impl GearModes {
    pub fn from_integer(v: u8) -> Self {
        match v {
            0 => Self::P,
            1 => Self::R,
            2 => Self::N,
            3 => Self::D,
            _ => panic!("Unknown value: {}", v),
        }
    }
}

impl From<u8> for GearModes {
    fn from(t: u8) -> GearModes {
        assert!(Self::P as u8 <= t && t <= Self::D as u8);
        unsafe { transmute(t) }
    }
}

pub struct Actuator<'l> {
    motor_plus: Output<'l>,
    motor_minus: Output<'l>,
    #[allow(dead_code)]
    // TODO: Remove once we've figured out how to move the actuator to specific location.
    v_select: Output<'l>,
    feedback: Channel<'l>,
    adc: Adc<'l, Async>,
    //phantom: PhantomData<&'l PotPin>,
}

impl<'l> Actuator<'l> {
    pub fn new(
        pin_motor_plus: Peri<'l, AnyPin>,
        pin_motor_minus: Peri<'l, AnyPin>,
        pin_volt_select: Peri<'l, AnyPin>,
        pot_pin: Peri<'l, impl AdcPin + 'l>,
        adc: Peri<'l, ADC>,
        irqs: impl Binding<ADC_IRQ_FIFO, InterruptHandler>,
    ) -> Self {
        // -----
        // Initialize the motor pins.
        let mut actuator_motor_plus = Output::new(pin_motor_plus, Level::Low);
        let mut actuator_motor_minus = Output::new(pin_motor_minus, Level::Low);
        let mut actuator_voltage_select = Output::new(pin_volt_select, Level::Low); // Low=12V, High=5V

        // -----
        // Initialize the potentiometer pin.
        let adc = Adc::new(adc, irqs, Config::default());
        let actuator_potentiometer = Channel::new_pin(pot_pin, Pull::None);

        // -----
        // Setup the motor pins.

        // https://wiki.purduesigbots.com/electronics/general/output-drive
        // https://docs.embassy.dev/embassy-rp/0.8.0/rp2040/gpio/enum.Drive.html
        actuator_motor_plus.set_drive_strength(Drive::_8mA);
        actuator_motor_minus.set_drive_strength(Drive::_8mA);
        actuator_voltage_select.set_drive_strength(Drive::_8mA);

        // https://wiki.purduesigbots.com/electronics/general/slew-rate
        // https://docs.embassy.dev/embassy-rp/0.8.0/rp2040/gpio/enum.SlewRate.html
        actuator_motor_plus.set_slew_rate(SlewRate::Fast);
        actuator_motor_minus.set_slew_rate(SlewRate::Fast);
        actuator_voltage_select.set_slew_rate(SlewRate::Fast);

        // -----
        // Output some basic info we're using.
        info!(
            "[actuator] Distance between gear positions={}mm; Allowed gear difference={}mm/{}Ω",
            DISTANCE_BETWEEN_POSITIONS,
            DISTANCE_ALLOWED_GEAR_DIFFERENCE,
            RESISTANCE_ALLOWED_GEAR_DIFFERENCE
        );
        info!(
            "[actuator] Throw resistance min={}Ω; Throw resistance max={}Ω; Throw resistance total={}Ω",
            RESISTANCE_THROW_MIN, RESISTANCE_THROW_MAX, RESISTANCE_THROW_TOTAL
        );
        info!(
            "[actuator] Throw resistance/mm={}Ω; Throw resistance/gear={}Ω",
            RESISTANCE_THROW_1MM, RESISTANCE_PER_GEAR
        );
        info!(
            "[actuator] Throw time/mm={}ms; Trow time/gear={}ms",
            TIME_THROW_1MM, TIME_THROW_GEAR
        );
        info!(
            "[actuator] Positions: GearP={:?}Ω; GearR={:?}Ω; GearN={:?}Ω; GearD={:?}Ω",
            GEAR_P, GEAR_R, GEAR_N, GEAR_D
        );

        // -----
        // Initialize our struct.
        Self {
            motor_plus: actuator_motor_plus,
            motor_minus: actuator_motor_minus,
            v_select: actuator_voltage_select,
            feedback: actuator_potentiometer,
            adc: adc,
            //phantom: PhantomData,
        }
    }

    // Test actuator control. Move it forward 2mm, then backward 2mm.
    // This *should* be safe to do EVEN IF (!!) we're moving (for whatever reason).
    // Returns:
    //   * TRUE  => Have moved. As in, it moved the distance back and forth and then returned.
    //   * FALSE => Have not moved.
    pub async fn test_actuator(&mut self) -> bool {
        info!("[actuator] Testing actuator control");

        let test_distance: u16 = 5;

        // ===== Verify that the actuator works by moving it back and forth.

        // Read start position (Ω).
        let position_start: u16 = self.read_pot().await;
        trace!(
            "[actuator] test_actuator(): Actuator test position (start): {}",
            position_start
        );

        // ----- Move actuator Xmm forward.
        debug!(
            "[actuator] Test Actuator: Move actuator {}mm forward",
            test_distance
        );
        if !self
            .move_actuator(RESISTANCE_THROW_1MM * test_distance, Direction::Forward)
            .await
        {
            // Did not move - return test failure.
            error!("[actuator] Actuator have not moved - test failure (#1/3)");
            return false;
        }
        Timer::after_millis(250).await;

        // ----- Move actuator Xmm backward.
        debug!(
            "[actuator] Test Actuator: Move actuator {}mm backward",
            test_distance
        );
        if !self
            .move_actuator(RESISTANCE_THROW_1MM * test_distance, Direction::Backward)
            .await
        {
            // Did not move - return test failure.
            error!("[actuator] Actuator have not moved - test failure (#2/3)");
            return false;
        }

        // Read end position. Should be the same as start position..
        let position_end: u16 = self.read_pot().await;
        trace!(
            "[actuator] test_actuator(): Actuator test position (end): {:?}",
            position_end
        );

        // Verify overall move.
        let moved = self.verify_moved(position_start, position_end);
        info!(
            "[actuator] Verifying total move ({}/{}): {}",
            position_start, position_end, moved
        );
        if !moved {
            // Have moved (it didn't return to original position) - return test failure.
            error!("[actuator] Actuator have not moved - test failure (#3/3)");
            return false;
        }

        return true;
    }

    // Move the actuator to a specific gear mode position.
    pub async fn change_gear_mode(&mut self, mode: GearModes) {
        debug!("[actuator] change_gear_mode({})", mode);

        // Find current position - Ω.
        let current_position = self.read_pot().await;
        debug!(
            "[actuator] change_gear_mode(): Current position={}Ω",
            current_position
        );

        // Set position for new gear mode - Ω.
        let destination_position: u16;
        match mode {
            GearModes::P => destination_position = GEAR_P,
            GearModes::R => destination_position = GEAR_R,
            GearModes::N => destination_position = GEAR_N,
            GearModes::D => destination_position = GEAR_D,
        }
        debug!(
            "[actuator] change_gear_mode(): Destination position={}Ω",
            destination_position
        );

        // Calculate how long to move the actuator - ms.
        let amount: u16 = ((destination_position as i16 - current_position as i16).abs()
            / RESISTANCE_THROW_1MM as i16) as u16;

        // .. in what direction.
        let direction: Direction;
        trace!("[actuator] change_gear_mode(): amount={}", amount);
        if current_position < destination_position {
            direction = Direction::Forward;
        } else {
            direction = Direction::Backward;
        }

        // Move the actuator into position.
        self.move_actuator(amount, direction).await;
    }

    // ==================================================
    // Private functions.
    // ==================================================

    // Check if after-move position is between pos1 max/min.
    // Returns:
    //   * TRUE  => Have moved.
    //   * FALSE => Have not moved.
    fn verify_moved(&mut self, before: u16, after: u16) -> bool {
        // NOTE: We only check that it HAVE moved, not with how much.
        // NOTE: Take a +-10Ω difference on the reading. The ADC in the Pico isn't very accurate.
        trace!(
            "[actuator] verify_moved(before={}, after={})",
            before,
            after
        );
        if before < 25 {
            debug!(
                "[actuator] Can't verify move, no resonable value from pot - return 'FALSE' (Have NOT moved)"
            );
            return false;
        }

        let before_min = before - RESISTANCE_ALLOWED_GEAR_DIFFERENCE;
        let before_max = before + RESISTANCE_ALLOWED_GEAR_DIFFERENCE;
        trace!(
            "[actuator] verify_moved(): (after({}) > before_min({})) && (after({}) < before_max({})); Diff={}",
            after,
            before_min,
            after,
            before_max,
            RESISTANCE_ALLOWED_GEAR_DIFFERENCE
        );

        if (after > before_min) && (after < before_max) {
            debug!("[actuator] Verify Moved: TRUE");
            return true;
        } else {
            debug!("[actuator] Verify Moved: FALSE");
            return false;
        }
    }

    // ==================================================
    // NOTE: These should really be private functions, but I need them to do the calibration.
    // ==================================================

    // Read the actuator potentiometer value.
    pub async fn read_pot(&mut self) -> u16 {
        let mut measurements = [0u16; READ_ACTUATOR_TIMES as usize];
        let mut pos = 0;
        let mut lowest: u16 = 0;
        let mut highest: u16 = 0;

        // Set both pins to LOW to brake the motor in the actuator.
        // Can't seem to read the pot while the actuator is moving.
        debug!("[actuator] Breaking actuator");
        self.motor_plus.set_low();
        self.motor_minus.set_low();

        debug!("[actuator] Reading actuator potentiometer value");
        loop {
            match self.adc.read(&mut self.feedback).await {
                Ok(val) => {
                    trace!("[actuator]   Read value: {:?}", val);

                    measurements[pos] = val;
                    pos = (pos + 1) % READ_ACTUATOR_TIMES as usize;
                    if pos == 0 {
                        // Compute average of measurements.
                        let average =
                            (measurements.iter().sum::<u16>() / READ_ACTUATOR_TIMES) as f32;

                        trace!(
                            "[actuator]   lowest: {:?}; highest: {:?}; average: {:?}",
                            lowest,
                            highest,
                            average as f32
                        );
                        return average as u16;
                    }

                    if lowest == 0 {
                        lowest = val;
                    } else if val < lowest {
                        lowest = val;
                    }
                    if val > highest {
                        highest = val;
                    }
                }
                Err(e) => {
                    error!(
                        "[actuator] Failed to read actuator porentiometer value: {:?}",
                        e
                    );
                    return 0;
                }
            }
        }
    }

    // Move the actuator - distance (in Ω).
    pub async fn move_actuator(&mut self, distance: u16, direction: Direction) -> bool {
        debug!("[actuator] Move actuator: {}Ω/{}", distance, direction);

        let position_end: i16;
        let mut position_now: u16;

        // Read the actuator feedback before we start moving.
        position_now = self.read_pot().await;
        debug!(
            "[actuator] Move Actuator: Pot value - before move: {}",
            position_now
        );

        // Where do we want it moved?
        if direction == Direction::Forward {
            position_end = (position_now as i16).abs() + distance as i16;
            debug!(
                "[actuator] position_end({}) = position_now({}) + {}",
                position_end,
                (position_now as i16).abs(),
                distance
            );
        } else {
            position_end = (position_now as i16).abs() - distance as i16;
            debug!(
                "[actuator] position_end({}) = position_now({}) - {}",
                position_end,
                (position_now as i16).abs(),
                distance
            );
        }
        debug!("[actuator] Final position wanted: {}", position_end);

        if (position_end >= RESISTANCE_THROW_TOTAL as i16) && (direction == Direction::Forward) {
            info!("[actuator] We're beyond total throw, return");
            return true;
        } else if (position_end <= RESISTANCE_THROW_MIN as i16)
            && (direction == Direction::Backward)
        {
            info!("[actuator] We're beyond minimum throw, return");
            return true;
        }

        let mut left2move: i16;
        loop {
            left2move = (position_end - position_now as i16).abs();
            debug!(
                "[actuator] Check move: want={} - now={} = left={}",
                position_end, position_now, left2move
            );

            // If we're very close (+/- 2mm), then we're in position.
            // NOTE: 1mm seems to cause a reset.
            if left2move <= (RESISTANCE_THROW_1MM as i16 * 2) {
                debug!("[actuator] We're within 2mm - DONE");
                break;

            // If we're close (+/- 10mm), then switch to 5V drive.
            } else if left2move <= (RESISTANCE_THROW_1MM as i16 * 10)
                && !self.v_select.is_set_high()
            {
                debug!("[actuator] Set level  5V");

                self.motor_plus.set_low();
                self.motor_minus.set_low();

                self.v_select.set_level(Level::High);

            // We have a long way left, switch to 12V drive.
            } else if left2move >= (RESISTANCE_THROW_1MM as i16 * 15) && !self.v_select.is_set_low()
            {
                debug!("[actuator] Set level 12V");

                self.motor_plus.set_low();
                self.motor_minus.set_low();

                self.v_select.set_level(Level::Low);
            }

            // Start moving the actuator in `direction`
            if direction == Direction::Forward {
                debug!("[actuator] Move actuator: direction=FORWARD");
                self.motor_plus.set_high();
            } else {
                debug!("[actuator] Move actuator: direction=BACKWARD");
                self.motor_minus.set_high();
            }

            // Check every 5ms
            Timer::after_millis(5).await;

            // Read the actuator feedback now that we've moved.
            position_now = self.read_pot().await;
            debug!(
                "[actuator] Move Actuator: Pot value - after move: {}",
                position_now
            );
        }

        return true;
    }
}
