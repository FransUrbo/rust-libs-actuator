#![no_std]

use defmt::{debug, error, info, trace, warn, Format};

use embassy_rp::adc::{Adc, AdcPin, Async, Channel, Config, InterruptHandler};
use embassy_rp::gpio::{AnyPin, Drive, Level, Output, Pull, SlewRate};
use embassy_rp::interrupt::typelevel::{Binding, ADC_IRQ_FIFO};
use embassy_rp::peripherals::ADC;
use embassy_rp::Peri;
use embassy_time::Timer;

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

// These are public, because if/when we need to test the actuator, we need to know them.
pub static TIME_THROW_1MM: f32 = (TIME_THROW_TOTAL as f32 / DISTANCE_THROW_TOTAL) as f32;
pub static TIME_THROW_GEAR: f32 = (TIME_THROW_1MM * DISTANCE_BETWEEN_POSITIONS) as f32;

pub static RESISTANCE_THROW_TOTAL: u16 = RESISTANCE_THROW_MAX - RESISTANCE_THROW_MIN;
pub static RESISTANCE_THROW_1MM: u16 =
    (RESISTANCE_THROW_TOTAL as f32 / DISTANCE_THROW_TOTAL) as u16;

// -----

static RESISTANCE_ALLOWED_GEAR_DIFFERENCE: u16 =
    (DISTANCE_ALLOWED_GEAR_DIFFERENCE as u16) * RESISTANCE_THROW_1MM;

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

    pub fn iterator() -> impl Iterator<Item = GearModes> {
        [Self::P, Self::R, Self::N, Self::D].iter().copied()
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
    v_select: Output<'l>,
    feedback: Channel<'l>,
    adc: Adc<'l, Async>,
}

impl<'l> Actuator<'l> {
    pub fn new(
        pin_motor_plus: Peri<'l, AnyPin>,
        pin_motor_minus: Peri<'l, AnyPin>,
        pin_volt_select: Peri<'l, AnyPin>,
        pin_pot: Peri<'l, impl AdcPin + 'l>,
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
        let actuator_potentiometer = Channel::new_pin(pin_pot, Pull::None);

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
        }
    }

    // ==================================================
    // Public functions.
    // ==================================================

    // Test actuator control. Move it forward 5mm, then backward 5mm.
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
            .move_actuator(position_start + (RESISTANCE_THROW_1MM * test_distance))
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
            .move_actuator(position_start - (RESISTANCE_THROW_1MM * test_distance))
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
        let moved = self.verify_moved(position_start, position_end).await;
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
    pub async fn change_gear_mode(&mut self, mode: GearModes) -> bool {
        debug!("[actuator] change_gear_mode({})", mode);

        // Set position for new gear mode - Ω.
        let destination: u16;
        match mode {
            GearModes::P => destination = GEAR_P,
            GearModes::R => destination = GEAR_R,
            GearModes::N => destination = GEAR_N,
            GearModes::D => destination = GEAR_D,
        }
        debug!(
            "[actuator] change_gear_mode(): Destination={}Ω",
            destination
        );

        // Move the actuator into position.
        return self.move_actuator(destination).await;
    }

    // ==================================================
    // Private functions.
    // ==================================================

    // Check if after-move position is between pos1 max/min.
    // Returns:
    //   * TRUE  => Have moved.
    //   * FALSE => Have not moved.
    async fn verify_moved(&mut self, before: u16, after: u16) -> bool {
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

    // Brake the actuator - set both pins to LOW.
    // NOTE: Does NOT like having this as an async!
    fn brake_actuator(&mut self) {
        if self.motor_plus.is_set_high() || self.motor_minus.is_set_high() {
            debug!("[actuator] Breaking actuator");
        }

        if self.motor_plus.is_set_high() {
            self.motor_plus.set_low();
        }

        if self.motor_minus.is_set_high() {
            self.motor_minus.set_low();
        }

        return;
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

    // Move the actuator to `position` (in Ω).
    pub async fn move_actuator(&mut self, position: u16) -> bool {
        let mut position_now: u16;
        let mut direction: Direction;

        debug!("[actuator] Move actuator to postition {}Ω", position);

        // Read the actuator feedback before we start moving.
        position_now = self.read_pot().await;
        debug!(
            "[actuator] Move Actuator: Pot value - before move: {}Ω",
            position_now
        );

        // Decide in what direction to go. If we're trying to go beyond the end, then return.
        if position_now < position {
            direction = Direction::Forward;

            if position > RESISTANCE_THROW_MAX {
                warn!("[actuator] We've been asked to go beyond maximum throw");
                return true; // TODO: Isn't this technically an error!?
            }
        } else {
            direction = Direction::Backward;

            if position < RESISTANCE_THROW_MIN {
                warn!("[actuator] We've been asked to go beyond minimum throw");
                return true; // TODO: Isn't this technically an error!?
            }
        }

        // We only need know how much left to move, so that we can switch between 12V and 5V drive.
        // NOTE: It doesn't seem to be necessary though, because I can get within less than 1mm anyway on 12V!
        let mut left2move: u16;
        loop {
            // Double check that the feedback value is sane.
            // However, we can get all sorts of random values here, so this isn't 100% :(.
            // But because we're doing it *inside* the loop, instead of at the entrence of the
            // function, we have a slightly higher chance of catching it..
            if position_now > (RESISTANCE_THROW_MAX + RESISTANCE_THROW_1MM)
                || position_now < (RESISTANCE_THROW_MIN - RESISTANCE_THROW_1MM)
            {
                error!("[actuator] Impossible reading - is the actuator connected?");
                debug!(
                    "[actuator] ({} > {}) || ({} < {})",
                    position_now,
                    (RESISTANCE_THROW_MAX + RESISTANCE_THROW_1MM),
                    position_now,
                    (RESISTANCE_THROW_MIN - RESISTANCE_THROW_1MM)
                );
                self.brake_actuator(); // Brake the actuator before we leave.
                return false;
            }

            left2move = ((position as i16 - position_now as i16).abs()) as u16;
            debug!(
                "[actuator] Check move: (want={}) - (now={}) = (left={})",
                position, position_now, left2move
            );

            trace!(
                "[actuator] Pin+:{:?}; Pin-:{:?}",
                self.motor_plus.is_set_high(),
                self.motor_minus.is_set_high()
            );

            // If we're VERY close (< 1mm/16Ω), then we're in position.
            if left2move < RESISTANCE_THROW_1MM {
                // TODO: We're still crashing here every now and then.
                //       Sometimes we get this debug, sometimes we get the one in `brake_actuator()`,
                //       sometimes neither. Sometimes we get "malformed frame skipped"!!
                debug!("[actuator] We're within +/- 1mm - DONE");
                self.brake_actuator(); // Brake the actuator before we leave.
                return true;

            // If we're close (< 5mm/80Ω), then switch to 5V drive.
            } else if left2move < (RESISTANCE_THROW_1MM * 5) && !self.v_select.is_set_high() {
                self.brake_actuator(); // Brake the actuator before we change the level.
                debug!("[actuator] Set level:  5V");
                self.v_select.set_high();

            // We have a long way to go (> 10mm/160Ω), switch to 12V drive.
            } else if left2move >= (RESISTANCE_THROW_1MM * 10) && !self.v_select.is_set_low() {
                self.brake_actuator(); // Brake the actuator before we change the level.
                debug!("[actuator] Set level: 12V");
                self.v_select.set_low();

            // We've gone beyond our desired position.
            } else if direction == Direction::Backward && position >= position_now {
                warn!("[actuator] We've gone beyond our desired position - go forward");
                self.brake_actuator(); // Brake the actuator before we change direction.
                direction = Direction::Forward;
            } else if direction == Direction::Forward && position <= position_now {
                warn!("[actuator] We've gone beyond our desired position - go backward");
                self.brake_actuator(); // Brake the actuator before we change direction.
                direction = Direction::Backward;
            }

            // Start moving the actuator in `direction`
            debug!("[actuator] Moving actuator: direction={}", direction);
            if direction == Direction::Forward && !self.motor_plus.is_set_high() {
                self.motor_plus.set_high();
            } else if direction == Direction::Backward && !self.motor_minus.is_set_high() {
                self.motor_minus.set_high();
            }

            // Read the actuator feedback now that we're moving.
            position_now = self.read_pot().await;
        }
    }
}
