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
//use heapless::Vec;

// ==============================
// Setup static values, measured on the actuator.

// The resistance on the actuator at MIN and MAX throw.
// Measured on the actuator at endpoints, using the `read-actuator-pot` application.
// The multimeter have different values though, so not sure exactly what value this is!
pub static RESISTANCE_THROW_MIN: u16 = 240; // Ω
pub static RESISTANCE_THROW_MAX: u16 = 1922; // Ω

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

static RESISTANCE_THROW_TOTAL: u16 = RESISTANCE_THROW_MAX - RESISTANCE_THROW_MIN;
static RESISTANCE_THROW_1MM: u16 = (RESISTANCE_THROW_TOTAL as f32 / DISTANCE_THROW_TOTAL) as u16;

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
    Forward,
    Backward,
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
    //gear_positions: Vec<u16, 4>,
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
        // Initialize the motor pins.
        let mut actuator_motor_plus = Output::new(pin_motor_plus, Level::Low);
        let mut actuator_motor_minus = Output::new(pin_motor_minus, Level::Low);
        let mut actuator_voltage_select = Output::new(pin_volt_select, Level::Low);

        // Initialize the potentiometer pin.
        let adc = Adc::new(adc, irqs, Config::default());
        let actuator_potentiometer = Channel::new_pin(pot_pin, Pull::None);

        // Setup the motor pins.
        // https://wiki.purduesigbots.com/electronics/general/output-drive
        actuator_motor_plus.set_drive_strength(Drive::_2mA);
        actuator_motor_minus.set_drive_strength(Drive::_2mA);
        actuator_voltage_select.set_drive_strength(Drive::_2mA);

        // https://wiki.purduesigbots.com/electronics/general/slew-rate
        actuator_motor_plus.set_slew_rate(SlewRate::Fast);
        actuator_motor_minus.set_slew_rate(SlewRate::Fast);
        actuator_voltage_select.set_slew_rate(SlewRate::Fast);

        // Output some basic info we're using.
        info!(
            "Distance between gear positions={}mm; Throw time/mm={}ms; Trow time/gear={}ms",
            DISTANCE_BETWEEN_POSITIONS, TIME_THROW_1MM, TIME_THROW_GEAR
        );
        info!(
            "Throw resistance/mm={}Ω; Throw resistance/gear={}Ω; Throw resistance/total={}Ω",
            RESISTANCE_THROW_1MM, RESISTANCE_PER_GEAR, RESISTANCE_THROW_TOTAL
        );
        info!(
            "Allowed gear difference={}mm/{}Ω",
            DISTANCE_ALLOWED_GEAR_DIFFERENCE, RESISTANCE_ALLOWED_GEAR_DIFFERENCE
        );
        info!(
            "Positions: GearP={:?}Ω; GearR={:?}Ω; GearN={:?}Ω; GearD={:?}Ω",
            GEAR_P, GEAR_R, GEAR_N, GEAR_D
        );

        //        // Setup a vector with our gear mode positions.
        //        let mut positions: Vec<u16, 4> = heapless::Vec::new(); // Return buffer.
        //        positions.push(GEAR_D).unwrap();
        //        positions.push(GEAR_N).unwrap();
        //        positions.push(GEAR_R).unwrap();
        //        positions.push(GEAR_P).unwrap();

        // Initialize our struct.
        Self {
            motor_plus: actuator_motor_plus,
            motor_minus: actuator_motor_minus,
            v_select: actuator_voltage_select,
            feedback: actuator_potentiometer,
            adc: adc,
            //gear_positions: positions,
            //phantom: PhantomData,
        }
    }

    // Test actuator control. Move it forward 2mm, then backward 2mm.
    // This *should* be safe to do EVEN IF (!!) we're moving (for whatever reason).
    // Returns:
    //   * TRUE  => Have moved. As in, it moved the distance back and forth and then returned.
    //   * FALSE => Have not moved.
    pub async fn test_actuator(&mut self) -> bool {
        info!("Testing actuator control");

        // ===== Verify that the actuator works by moving it back and forth.

        // Read start position (Ω).
        let position_start: u16 = self.read_pot().await;
        trace!(
            "test_actuator(): Actuator test position (start): {}",
            position_start
        );

        // ----- Move actuator 2mm forward.
        debug!("test_actuator(): Move actuator 2mm forward");
        if !self
            .move_actuator(TIME_THROW_1MM * 2.0, Direction::Forward)
            .await
        {
            // Did not move - return test failure.
            error!("Actuator have not moved - test failure (#1/3)");
            return false;
        }
        Timer::after_millis(250).await;

        // ----- Move actuator 2mm backward.
        debug!("test_actuator(): Move actuator 2mm backward");
        if !self
            .move_actuator(TIME_THROW_1MM * 2.0, Direction::Backward)
            .await
        {
            // Did not move - return test failure.
            error!("Actuator have not moved - test failure (#2/3)");
            return false;
        }

        // Read end position. Should be the same as start position..
        let position_end: u16 = self.read_pot().await;
        trace!(
            "test_actuator(): Actuator test position (end): {:?}",
            position_end
        );

        // Verify overall move.
        let moved = self.verify_moved(position_start, position_end);
        info!(
            "Verifying total move ({}/{}): {}",
            position_start, position_end, moved
        );
        if !moved {
            // Have moved (it didn't return to original position) - return test failure.
            error!("Actuator have not moved - test failure (#3/3)");
            return false;
        }

        return true;
    }

    // Move the actuator to a specific gear mode position.
    pub async fn change_gear_mode(&mut self, mode: GearModes) {
        debug!("change_gear_mode({})", mode);

        // Find current position - Ω.
        let current_position = self.read_pot().await;
        debug!("change_gear_mode(): Current position={}Ω", current_position);

        // Set position for new gear mode - Ω.
        let destination_position: u16;
        match mode {
            GearModes::P => destination_position = GEAR_P,
            GearModes::R => destination_position = GEAR_R,
            GearModes::N => destination_position = GEAR_N,
            GearModes::D => destination_position = GEAR_D,
        }
        debug!(
            "change_gear_mode(): Destination position={}Ω",
            destination_position
        );

        // Calculate how long to move the actuator - ms.
        let amount: f32 =
            (destination_position as i16 - current_position as i16).abs() as f32 / TIME_THROW_1MM;

        // .. in what direction.
        let direction: Direction;
        trace!("change_gear_mode(): amount={}", amount);
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

    //    fn find_closest_gear_mode(&self, target: u16) -> u16 {
    //        debug!("find_closest_gear_mode({})", target);
    //
    //        // Initialize variables to track the closest value and its difference
    //        let mut closest_value = self.gear_positions[0];
    //        let mut min_difference = u16::MAX;
    //
    //        // Iterate through the values and calculate the difference
    //        for mode in &self.gear_positions {
    //            trace!("let difference = {} - {}", mode, target);
    //            let difference = mode - target;
    //
    //            // Update the closest value if a smaller difference is found
    //            if difference < min_difference {
    //                min_difference = difference;
    //                closest_value = *mode;
    //            }
    //        }
    //
    //        return closest_value;
    //    }

    // Check if after-move position is between pos1 max/min.
    // Returns:
    //   * TRUE  => Have moved.
    //   * FALSE => Have not moved.
    fn verify_moved(&mut self, before: u16, after: u16) -> bool {
        // NOTE: We only check that it HAVE moved, not with how much.
        // NOTE: Take a +-10Ω difference on the reading. The ADC in the Pico isn't very accurate.
        trace!("verify_moved(before={}, after={})", before, after);
        if before < 25 {
            debug!(
                "Can't verify move, no resonable value from pot - return 'FALSE' (Have NOT moved)"
            );
            return false;
        }

        // TODO: This does not work if we're moving 2mm +/- in testing!
        //RESISTANCE_THROW_1MM * 2
        let before_min = before - RESISTANCE_ALLOWED_GEAR_DIFFERENCE;
        let before_max = before + RESISTANCE_ALLOWED_GEAR_DIFFERENCE;
        trace!(
            "verify_moved(): (after({}) > before_min({})) && (after({}) < before_max({})); Diff={}",
            after,
            before_min,
            after,
            before_max,
            RESISTANCE_ALLOWED_GEAR_DIFFERENCE
        );

        if (after > before_min) && (after < before_max) {
            debug!("verify_moved(): Moved=TRUE");
            return true;
        } else {
            debug!("verify_moved(): Moved=FALSE");
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

        debug!("Reading actuator potentiometer value");
        loop {
            // Settle the actuator before we read.
            // NOTE: CRASH - 25ms causes the Pico to crash!
            //       However, 50ms seems to work fine..
            trace!("read_pot(): Sleeping 50ms");
            Timer::after_millis(50).await;
            trace!("read_pot(): Sleeping 50ms - done");
            // NOTE: CRASH - Something that 'done' debug works!
            //       Could it be the `adc.read()` that fails!?
            //       BUT, only on the initial boot! If the Pico have
            //       crashed, it keeps crashing on the `adc.read()`.

            match self.adc.read(&mut self.feedback).await {
                Ok(val) => {
                    trace!("Actuator read value: {:?}", val);

                    measurements[pos] = val;
                    pos = (pos + 1) % READ_ACTUATOR_TIMES as usize;
                    if pos == 0 {
                        // Compute average of measurements.
                        let average =
                            (measurements.iter().sum::<u16>() / READ_ACTUATOR_TIMES) as f32;

                        debug!(
                            "Actuator lowest: {:?}; highest: {:?}; average: {:?}",
                            lowest, highest, average as f32
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
                    error!("Failed to read actuator porentiometer value: {:?}", e);
                    return 0;
                }
            }
        }
    }

    // Move the actuator - distance (in milliseconds).
    // TODO: This should probably be "to position" (Ω) instead of time.
    //         * Move Xms
    //         * Check the position (Ω)
    //         * If it's not within +/- DISTANCE_ALLOWED_GEAR_DIFFERENCE
    //           then keep moving (or change direction).
    //       OR
    //       Keep this, but call it from `change_gear_mode()` and `test_actuator()` etc in short bursts,
    //       and *there* check the position (within +/- DISTANCE_ALLOWED_GEAR_DIFFERENCE).
    //       BUT, then we need to return Ω here instead.
    pub async fn move_actuator(&mut self, distance: f32, direction: Direction) -> bool {
        debug!("Moving actuator: {}/{}", distance, direction);

        // Set both pins to LOW to brake the motor in the actuator.
        // NOTE: The Arduino example say to set them HIGH, but that will turn ON the relays! (??)
        // https://www.progressiveautomations.com/blogs/how-to/how-to-use-relays-to-control-linear-actuators
        self.motor_plus.set_low();
        self.motor_minus.set_low();

        // Read the actuator feedback before we start moving.
        let position_start = self.read_pot().await;
        debug!(
            "move_actuator(): Actuator potentiometer value - before move: {}",
            position_start
        );

        // Move actuator for Xms in `direction`.
        // TODO: It seems the timer crashes if the distance in ms is < 50ms!??!
        if direction == Direction::Forward {
            debug!(
                "move_actuator(): Move actuator: direction=FORWARD; distance={}ms",
                distance
            );

            trace!("move_actuator(): (F1/4)");
            self.motor_plus.set_high();
            trace!("move_actuator(): (F2/4) - distance={}ms", distance);
            Timer::after_millis(distance as u64).await; // TODO: CRASH!!
            trace!("move_actuator(): (F3/4)");
            self.motor_plus.set_low();
            trace!("move_actuator(): (F4/4)");
        } else {
            debug!(
                "move_actuator(): Move actuator: direction=BACKWARD; distance={}ms",
                distance
            );

            trace!("move_actuator(): (B1/4)");
            self.motor_minus.set_high();
            trace!("move_actuator(): (B2/4) - distance={}ms", distance);
            Timer::after_millis(distance as u64).await; // TODO: CRASH!!
            trace!("move_actuator(): (B3/4)");
            self.motor_minus.set_low();
            trace!("move_actuator(): (B4/4)");
        }

        // Read the actuator feedback now that we've moved.
        trace!("move_actuator(): Reading pot..");
        let position_end = self.read_pot().await;
        debug!(
            "move_actuator(): Actuator potentiometer value - after move: {}",
            position_end
        );

        // Verify with the potentiometer on the actuator that we've actually moved
        // it to the right position.
        return self.verify_moved(position_start, position_end);
    }
}
