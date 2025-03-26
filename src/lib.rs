#![no_std]

use defmt::{debug, error, trace, Format};

use embassy_rp::adc::{Adc, AdcPin, Async, Channel, Config, InterruptHandler};
use embassy_rp::gpio::{AnyPin, Drive, Level, Output, Pull, SlewRate};
use embassy_rp::interrupt::typelevel::{Binding, ADC_IRQ_FIFO};
use embassy_rp::peripherals::ADC;
use embassy_rp::Peripheral;
use embassy_time::Timer;

use core::marker::PhantomData;

// ==============================
// Setup static values, measured on the actuator.

// The resistance on the actuator at MIN and MAX throw.
// Measured on the actuator at endpoints, using the `read-actuator-pot` application.
// The multimeter have different values though, so not sure exactly what value this is!
pub static RESISTANCE_THROW_MIN: u16 = 240; // Ω
pub static RESISTANCE_THROW_MAX: u16 = 1922; // Ω

// Measured values:
// 'P' starts 10mm from outer endpoint.
//   * 'P': 1630Ω
//   * 'R': 1230Ω
//   * 'N':  830Ω
//   * 'D':  430Ω
static RESISTANCE_PER_GEAR: u16 = 400;

// Where the first gear mode is located (Ω from endpoint).
static RESISTANCE_ACTUATOR_START: u16 = 290;

// -----

// The total throw from MIN to MAX is `101.56mm` (`3.9984"`).
static DISTANCE_THROW_TOTAL: f32 = 101.56; // mm

// The distance between 'P' and 'D' is 70mm.
static DISTANCE_BETWEEN_POSITIONS: f32 = (70 / 3) as f32; // mm

// The allowed +/- value between MIN and MAX gear position is +/- 4mm.
static DISTANCE_ALLOWED_GEAR_DIFFERENCE: u8 = 4; // mm

// -----

// Moving the actuator between endpoints takes about 2s@12V.
// Make it public, so we can use it to test the actuator.
pub static TIME_THROW_TOTAL: u64 = 2350; // ms

// These two are public, because if/when we need to test the actuator, we need to
// know them..
pub static TIME_THROW_1MM: u16 = (TIME_THROW_TOTAL as f32 / DISTANCE_THROW_TOTAL as f32) as u16;
pub static TIME_THROW_GEAR: u64 =
    (TIME_THROW_1MM as f32 * DISTANCE_BETWEEN_POSITIONS as f32) as u64;

// ==============================
// Calculate values based on the static's..

static RESISTANCE_THROW_TOTAL: u16 = RESISTANCE_THROW_MAX - RESISTANCE_THROW_MIN;
static RESISTANCE_THROW_1MM: u16 = (RESISTANCE_THROW_TOTAL as f32 / DISTANCE_THROW_TOTAL) as u16;

static RESISTANCE_ALLOWED_GEAR_DIFFERENCE: u16 =
    (DISTANCE_ALLOWED_GEAR_DIFFERENCE as u16) * RESISTANCE_THROW_1MM;

// -----

// Pre-calculate the proper gear mode positions, based on the start position
// (which is where the 'D' mode is - max pulled in).
static GEAR_D: u16 = RESISTANCE_ACTUATOR_START + (RESISTANCE_PER_GEAR * 1); // Button::D
static GEAR_N: u16 = RESISTANCE_ACTUATOR_START + (RESISTANCE_PER_GEAR * 2); // Button::N
static GEAR_R: u16 = RESISTANCE_ACTUATOR_START + (RESISTANCE_PER_GEAR * 3); // Button::R
static GEAR_P: u16 = RESISTANCE_ACTUATOR_START + (RESISTANCE_PER_GEAR * 4); // Button::P

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

pub struct Actuator<'l, PotPin: AdcPin> {
    motor_plus: Output<'l>,
    motor_minus: Output<'l>,
    v_select: Output<'l>,
    feedback: Channel<'l>,
    adc: Adc<'l, Async>,
    phantom: PhantomData<&'l PotPin>,
}

impl<'l, PotPin: AdcPin> Actuator<'l, PotPin> {
    pub fn new(
        pin_motor_plus: AnyPin,
        pin_motor_minus: AnyPin,
        pin_volt_select: AnyPin,
        pot_pin: PotPin,
        adc: impl Peripheral<P = ADC> + 'l,
        irqs: impl Binding<ADC_IRQ_FIFO, InterruptHandler>,
    ) -> Self {
        // Initialize the motor pins.
        let mut actuator_motor_plus = Output::new(pin_motor_plus, Level::Low);
        let mut actuator_motor_minus = Output::new(pin_motor_minus, Level::Low);
        let mut actuator_voltage_select = Output::new(pin_volt_select, Level::Low);

        // Initialize the potentiometer pin.
        let adc = Adc::new(adc, irqs, Config::default());
        let actuator_potentiometer = Channel::new_pin(pot_pin, Pull::Down);

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
        trace!(
            "new(): Distance between gear positions={}mm; Throw time/mm={}ms; Trow time/gear={}ms",
            DISTANCE_BETWEEN_POSITIONS,
            TIME_THROW_1MM,
            TIME_THROW_GEAR
        );
        trace!(
            "new(): Throw resistance/mm={}Ω; Throw resistance/gear={}Ω",
            RESISTANCE_THROW_1MM,
            RESISTANCE_PER_GEAR
        );
        trace!(
            "new(): Allowed gear difference (distance)={}mm; Allowed gear difference (resistance)={}Ω",
            DISTANCE_ALLOWED_GEAR_DIFFERENCE,
            RESISTANCE_ALLOWED_GEAR_DIFFERENCE
        );

        // Initialize our struct.
        Self {
            motor_plus: actuator_motor_plus,
            motor_minus: actuator_motor_minus,
            v_select: actuator_voltage_select,
            feedback: actuator_potentiometer,
            adc: adc,
            phantom: PhantomData,
        }
    }

    // Test actuator control. Move it backward 2mm, then forward 2mm.
    // This *should* be safe to do EVEN IF (!!) we're moving (for whatever reason).
    // Returns:
    //   * TRUE  => Have moved. As in, it moved the distance back and forth and then returned.
    //   * FALSE => Have not moved.
    pub async fn test_actuator(&mut self) -> bool {
        debug!("Testing actuator control");

        // Read start position (Ω).
        let position_1: u16 = self.read_pot().await;
        trace!(
            "test_actuator(): Actuator test position (#1): {}",
            position_1
        );

        // Move actuator 2mm forward.
        self.move_actuator((TIME_THROW_1MM as u64) * 2, Direction::Forward)
            .await;

        // Give it 1/10s to settle.
        Timer::after_millis(100).await;

        // Read position.
        let position_2: u16 = self.read_pot().await;
        trace!(
            "test_actuator(): Actuator test position (#2): {}",
            position_2
        );

        // Make sure the actuator moved.
        if !self.verify_moved(position_1, position_2) {
            // Did not move - return test failure.
            return false;
        }

        // Move the actuator 2mm backward.
        self.move_actuator((TIME_THROW_1MM as u64) * 2, Direction::Backward)
            .await;

        // Give it 1/10s to settle.
        Timer::after_millis(100).await;

        // Read end position. Should be the same as start position..
        let position_3: u16 = self.read_pot().await;
        trace!(
            "test_actuator(): Actuator test position (#3): {}",
            position_3
        );

        // =====
        // Verify overall move.
        if self.verify_moved(position_1, position_3) {
            // Have moved (it didn't return to original position) - return test failure.
            return false;
        }

        // =====
        // The actuator worked, find what gear we're in.
        if self.find_gear().await == 4 {
            // UNSET. As in, we're in between gears!
            // NOTE: We know where the actuator is *now* (`position_3`).
            //       What is the closest gear mode from here?
            if position_3 < (GEAR_R + ((GEAR_R - GEAR_P) / 2)) {
                // Move to 'P'.
                trace!(
                    "test_actuator(): position_3={} < {}",
                    position_3,
                    (GEAR_R + ((GEAR_R - GEAR_P) / 2))
                );
                self.move_to_exact_position(0).await; // Button::P
            } else if position_3 < (GEAR_N + ((GEAR_N - GEAR_R) / 2)) {
                // Move to 'R'
                trace!(
                    "test_actuator(): position_3={} < {}",
                    position_3,
                    (GEAR_N + ((GEAR_N - GEAR_R) / 2))
                );
                self.move_to_exact_position(1).await; // Button::R
            } else if position_3 < (GEAR_D + ((GEAR_D - GEAR_N) / 2)) {
                // Move to 'N'
                trace!(
                    "test_actuator(): position_3={} < {}",
                    position_3,
                    (GEAR_D + ((GEAR_D - GEAR_N) / 2))
                );
                self.move_to_exact_position(2).await; // Button::N
            } else {
                // Move to 'D'
                trace!("test_actuator(): position_3={} (else)", position_3);
                self.move_to_exact_position(3).await; // Button::D
            }

            return true; // For now, just return success.
        }

        return true;
    }

    // Move the actuator to a specific gear mode position.
    // TODO: Remove the `fake` param as soon as the actuator works as intended.
    pub async fn change_gear_mode(&mut self, mode: u8, fake: i8) {
        // Get the resistance feedback value from the actuator.
        let current_gear = self.find_gear().await;
        debug!(
            "change_gear_mode(): Found current gear mode: {:?}",
            current_gear
        );
        let current_gear = fake; // TODO: Remove when actuator work as intended.

        // Calculate the amount of gears and direction to move.
        let gears: i8 = current_gear - mode as i8;

        // How long to keep the pin HIGH to move to the designated gear position.
        let move_time = TIME_THROW_GEAR * ((gears as i64).abs() as u64);
        trace!(
            "test_actuator(): Move move_time='{}ms * {}gears = {}ms'",
            TIME_THROW_GEAR,
            ((gears as i64).abs() as u64),
            move_time
        );

        // Move the actuator to the new gear mode position.
        if gears < 0 {
            self.move_actuator(move_time, Direction::Backward).await;
        } else {
            self.move_actuator(move_time, Direction::Forward).await;
        }
    }

    // ==================================================
    // Private functions.
    // ==================================================

    // Read the actuator potentiometer and translate the resistance from it to a gear mode positon.
    async fn find_gear(&mut self) -> i8 {
        // Get the actuator potentiometer value - ACTUAL position of the actuator.
        // Manufacturer say between 0-10kΩ, depending on throw distance.
        let pot = self.read_pot().await;

        // Sanity check, just to make sure we don't get a value that is LOWER than the first mode.
        if pot < RESISTANCE_ACTUATOR_START {
            debug!("Can't find gear, no resonable value from pot - return 'Button::UNSET'");
            return 4; // Button::UNSET
        }

        // Calculate what gear is in by the actual actuator potentiometer value.
        let current_gear_mode: u16 = (pot - RESISTANCE_ACTUATOR_START).try_into().unwrap();
        trace!(
            "find_gear(): gear={}; GEAR_P={}; GEAR_R={}; GEAR_N={}; GEAR_D={}",
            current_gear_mode,
            GEAR_P,
            GEAR_R,
            GEAR_N,
            GEAR_D
        );

        // Find the gear mode position, within a fault tolerance of the actuator
        // potentiometer and reading (the Pico ADC isn't very precise).
        if self.check_gear_position(current_gear_mode, GEAR_P) {
            0 // Button::P
        } else if self.check_gear_position(current_gear_mode, GEAR_R) {
            1 // Button::R
        } else if self.check_gear_position(current_gear_mode, GEAR_N) {
            2 // Button::N
        } else if self.check_gear_position(current_gear_mode, GEAR_D) {
            3 // Button::D
        } else {
            4 // Button::UNSET
        }
    }

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

        let before_min = before - RESISTANCE_ALLOWED_GEAR_DIFFERENCE;
        let before_max = before + RESISTANCE_ALLOWED_GEAR_DIFFERENCE;
        trace!(
            "verify_moved(): (after({}) > before_min({})) && (after({}) < before_max({}))",
            after,
            before_min,
            after,
            before_max
        );

        if (after > before_min) && (after < before_max) {
            trace!("verify_moved(): FALSE (Have NOT moved)");
            return false;
        } else {
            trace!("verify_moved(): TRUE (HAVE moved)");
            return true;
        }
    }

    // Simplify the check if a value falls within a certain tolerance.
    fn check_gear_position(&mut self, check: u16, validate: u16) -> bool {
        if (check > (validate - RESISTANCE_ALLOWED_GEAR_DIFFERENCE))
            && (check < (validate + RESISTANCE_ALLOWED_GEAR_DIFFERENCE))
        {
            return true;
        } else {
            return false;
        }
    }

    // Given a gear mode position, move in small steps until the actuator potentiometer
    // roughly matches (within `RESISTANCE_ALLOWED_GEAR_DIFFERENCE`Ω) that gear mode.
    // TODO: Should we even have a specific position to move to, why not just go to 'P'?
    // TODO: With a third relay, we can choose between +5V and +12V,
    //       and switch to the +5V instead, which will move the actuator
    //       slower.
    async fn move_to_exact_position(&mut self, mode: u8) -> bool {
        // Just define the variable..
        let mut dest: u16 = 0;

        // The way we calculate before calling this function, is that we're to far
        // *behind* (backward) of the position we want to go, so we move *forward*.
        let mut direction = Direction::Forward;

        // Start with 1/10s.
        let mut move_time: u64 = 100;

        // Find out what resistance the potentiometer should read when we're done.
        if mode == 0 {
            // Button::P
            dest = GEAR_P;
        } else if mode == 1 {
            // Button::R
            dest = GEAR_R;
        } else if mode == 2 {
            // Button::N
            dest = GEAR_N;
        } else if mode == 3 {
            // Button::D
            dest = GEAR_D;
        }
        debug!(
            "move_to_exact_position(mode={}): Wanted position: {}Ω",
            mode, dest
        );

        // Keep moving the actuator until we've arrived at the destination.
        loop {
            // We read the potentiometer at the *end*, because we already know we're off..

            // Move actuator.
            if direction == Direction::Forward {
                self.motor_plus.set_high();
            } else {
                self.motor_minus.set_high();
            }
            Timer::after_millis(move_time).await;

            // Stop actuator. Simpler to just put both LOW, than figure out which one..
            self.motor_plus.set_low();
            self.motor_minus.set_low();

            // Read the new position.
            let position: u16 = self.read_pot().await;
            trace!(
                "move_to_exact_position(): new actuator potentiometer value: {}",
                position
            );

            // Check if the new position is near the value of the gear mode we wanted.
            if self.check_gear_position(position, dest) {
                // We're done, we're at the desired gear mode.
                // Within +/- `RESISTANCE_ALLOWED_GEAR_DIFFERENCE`Ω
                self.v_select.set_low(); // Go back to using +12V for the actuator.
                return true;
            } else if position > dest {
                // We moved too far, change the direction and lower the move time.
                if direction == Direction::Forward {
                    direction = Direction::Backward;
                } else {
                    direction = Direction::Forward;
                }
                move_time = move_time / 2 as u64;
                self.v_select.set_high(); // Switch to +5V to make the actuator move slower.
            } // else: keep moving.
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

        loop {
            match self.adc.read(&mut self.feedback).await {
                Ok(val) => {
                    trace!("Actuator read value: {:?}", val);

                    measurements[pos] = val;
                    pos = (pos + 1) % READ_ACTUATOR_TIMES as usize;
                    if pos == 0 {
                        // Compute average of measurements.
                        let average = measurements.iter().sum::<u16>() / READ_ACTUATOR_TIMES;

                        trace!(
                            "Actuator lowest: {:?}; highest: {:?}; average: {:?}",
                            lowest, highest, average
                        );
                        return average;
                    }

                    if lowest == 0 {
                        lowest = val;
                    } else if val < lowest {
                        lowest = val;
                    }
                    if val > highest {
                        highest = val;
                    }

                    Timer::after_millis(50).await;
                }
                Err(e) => {
                    error!("Failed to read actuator porentiometer value: {:?}", e);
                    return 0;
                }
            }
        }
    }

    // Move the actuator - distance.
    pub async fn move_actuator(&mut self, distance: u64, direction: Direction) -> bool {
        // Set both pins to LOW to brake the motor in the actuator.
        // NOTE: The Arduino example say to set them HIGH, but that will turn ON the relays! (??)
        // https://www.progressiveautomations.com/blogs/how-to/how-to-use-relays-to-control-linear-actuators
        self.motor_plus.set_low();
        self.motor_minus.set_low();

        // Start by getting a reading of the actuator potentiometer before we start moving it.
        let actuator_pot_1 = self.read_pot().await;
        debug!(
            "Actuator potentiometer value - before move: {}",
            actuator_pot_1
        );

        if direction == Direction::Forward {
            debug!(
                "Moving actuator: direction=FORWARD; distance={}ms",
                distance
            );

            trace!("move_actuator(): (F1/4)");
            self.motor_plus.set_high();
            trace!("move_actuator(): (F2/4)");
            Timer::after_millis(distance).await;
            trace!("move_actuator(): (F3/4)");
            self.motor_plus.set_low();
            trace!("move_actuator(): (F4/4)");
        } else {
            debug!(
                "Moving actuator: direction=BACKWARD; distance={}ms",
                distance
            );

            trace!("move_actuator(): (B1/4)");
            self.motor_minus.set_high();
            trace!("move_actuator(): (B2/4)");
            Timer::after_millis(distance).await;
            trace!("move_actuator(): (B3/4)");
            self.motor_minus.set_low();
            trace!("move_actuator(): (B4/4)");
        }

        // TODO: Verify with the potentiometer on the actuator that we've actually moved
        //       it to the right position.
        trace!("move_actuator(): (R1/3)");
        Timer::after_millis(50).await;

        trace!("move_actuator(): (R2/3)");
        let actuator_pot_2 = self.read_pot().await;
        trace!("move_actuator(): (R3/3)");

        debug!(
            "Actuator potentiometer value - after move: {}",
            actuator_pot_2
        );

        return self.verify_moved(actuator_pot_1, actuator_pot_2);
    }
}
