#![no_std]

use defmt::{debug, trace, Format};

use embassy_rp::adc::{Adc, AdcPin, Async, Channel, Config, InterruptHandler};
use embassy_rp::gpio::{AnyPin, Drive, Level, Output, Pull, SlewRate};
use embassy_rp::interrupt::typelevel::{Binding, ADC_IRQ_FIFO};
use embassy_rp::peripherals::ADC;
use embassy_rp::Peripheral;
use embassy_time::Timer;

use core::marker::PhantomData;

// =====
// Setup static values, measured on the actuator.

// The resistance on the actuator at MIN and MAX throw.
// Measured on the actuator at endpoints, using the `read-actuator-pot` application.
// The multimeter have different values though, so not sure exactly what value this is!
static RESISTANCE_MIN_THROW: u16 = 245; // Ω
static RESISTANCE_MAX_THROW: u16 = 1800; // Ω

// Measured values:
// 'P' starts 10mm from outer endpoint.
//   * 'P': 1630Ω
//   * 'R': 1230Ω
//   * 'N':  830Ω
//   * 'D':  430Ω
static RESISTANCE_PER_GEAR: u16 = 400;

// The total throw from MIN to MAX is `101.56mm` (`3.9984"`).
static TOTAL_THROW_DISTANCE: f32 = 101.56; // mm

// Moving the actuator between endpoints takes about 2s@12V.
static TOTAL_THROW_TIME: u16 = 2350; // ms

// The distance between 'P' and 'D' is 70mm.
static DISTANCE_BETWEEN_POSITIONS: f32 = (70 / 3) as f32; // mm

// The allowed +/- value between MIN and MAX gear position is +/- 4mm.
static ALLOWED_GEAR_DIFFERENCE_DISTANCE: u8 = 4; // mm

// =====
// These are the actual information we're interested in.
static TOTAL_THROW_RESISTANCE: u16 = RESISTANCE_MAX_THROW - RESISTANCE_MIN_THROW;
static THROW_RESISTANCE_PER_1MM: u16 =
    (TOTAL_THROW_RESISTANCE as f32 / TOTAL_THROW_DISTANCE) as u16;

pub static THROW_TIME_PER_1MM: u16 = (TOTAL_THROW_TIME as f32 / TOTAL_THROW_DISTANCE as f32) as u16;
pub static THROW_TIME_PER_GEAR: u64 =
    (THROW_TIME_PER_1MM as f32 * DISTANCE_BETWEEN_POSITIONS as f32) as u64;

static ALLOWED_GEAR_DIFFERENCE_RESISTANCE: u16 =
    (ALLOWED_GEAR_DIFFERENCE_DISTANCE as u16) * THROW_RESISTANCE_PER_1MM;

#[derive(Copy, Clone, Format, PartialEq)]
#[repr(u8)]
pub enum Direction {
    Forward,
    Backward,
}

// =====

pub struct Actuator<'l, PotPin: AdcPin> {
    motor_plus: Output<'l>,
    motor_minus: Output<'l>,
    feedback: Channel<'l>,
    adc: Adc<'l, Async>,
    phantom: PhantomData<&'l PotPin>,
}

impl<'l, PotPin: AdcPin> Actuator<'l, PotPin> {
    pub fn new(
        pin_motor_plus: AnyPin,
        pin_motor_minus: AnyPin,
        pot_pin: PotPin,
        adc: impl Peripheral<P = ADC> + 'l,
        irqs: impl Binding<ADC_IRQ_FIFO, InterruptHandler>,
    ) -> Self {
        // Initialize the motor pins.
        let mut actuator_motor_plus = Output::new(pin_motor_plus, Level::Low);
        let mut actuator_motor_minus = Output::new(pin_motor_minus, Level::Low);

        // Initialize the potentiometer pin.
        let adc = Adc::new(adc, irqs, Config::default());
        let actuator_potentiometer = Channel::new_pin(pot_pin, Pull::Down);

        // Setup the motor pins.
        // https://wiki.purduesigbots.com/electronics/general/output-drive
        actuator_motor_plus.set_drive_strength(Drive::_2mA);
        actuator_motor_minus.set_drive_strength(Drive::_2mA);

        // https://wiki.purduesigbots.com/electronics/general/slew-rate
        actuator_motor_plus.set_slew_rate(SlewRate::Fast);
        actuator_motor_minus.set_slew_rate(SlewRate::Fast);

        // Output some basic info we're using.
        trace!(
            "new(): Distance between gear positions={}mm; Throw time/mm={}ms; Trow time/gear={}ms",
            DISTANCE_BETWEEN_POSITIONS,
            THROW_TIME_PER_1MM,
            THROW_TIME_PER_GEAR
        );
        trace!(
            "new(): Throw resistance/mm={}Ω; Throw resistance/gear={}Ω",
            THROW_RESISTANCE_PER_1MM,
            RESISTANCE_PER_GEAR
        );
        trace!(
            "new(): Allowed gear difference (distance)={}mm; Allowed gear difference (resistance)={}Ω",
            ALLOWED_GEAR_DIFFERENCE_DISTANCE,
            ALLOWED_GEAR_DIFFERENCE_RESISTANCE
        );

        // Initialize our struct.
        Self {
            motor_plus: actuator_motor_plus,
            motor_minus: actuator_motor_minus,
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
        self.move_actuator((THROW_TIME_PER_1MM as u64) * 2, Direction::Forward)
            .await;

        // Read position.
        let position_2: u16 = self.read_pot().await;
        trace!(
            "test_actuator(): Actuator test position (#2): {}",
            position_2
        );

        // Give it 1/10s to settle.
        Timer::after_millis(100).await;

        // Make sure the actuator moved.
        if !self.verify_moved(position_1, position_2).await {
            // Return test failure.
            return false;
        }

        // Move the actuator 2mm backward.
        self.move_actuator((THROW_TIME_PER_1MM as u64) * 2, Direction::Backward)
            .await;

        // Read end position. Should be the same as start position..
        let position_3: u16 = self.read_pot().await;
        trace!(
            "test_actuator(): Actuator test position (#3): {}",
            position_3
        );

        // Verify overall move.
        // Should be NO - have NOT moved between #1 and #3! As in, returned to original position.
        if !self.verify_moved(position_1, position_3).await {
            // Return test success.
            return true;
        } else {
            // Return test failure.
            return false;
        }
    }

    pub async fn change_gear_mode(&mut self, gears: i8) {
        // How long to keep the pin HIGH to move to the designated gear position.
        let move_time = THROW_TIME_PER_GEAR * ((gears as i64).abs() as u64);
        trace!(
            "test_actuator(): Move move_time='{}ms * {}gears = {}ms'",
            THROW_TIME_PER_GEAR,
            ((gears as i64).abs() as u64),
            move_time
        );

        if gears < 0 {
            self.move_actuator(move_time, Direction::Backward).await;
        } else {
            self.move_actuator(move_time, Direction::Forward).await;
        }
    }

    // Find the current gear.
    pub async fn find_gear(&mut self, start: u16) -> i8 {
        // Get the actuator potentiometer value - ACTUAL position of the actuator.
        // Manufacturer say between 0-10kΩ.
        let pot = self.read_pot().await;
        if pot < start {
            debug!("Can't find gear, no resonable value from pot - return 'Button::UNSET'");
            return 4;
        }

        // Distance positions. Ok, so this is dumb, but it's clear. And will be clear to me,
        // five years from now when/if I go back in here and needs to fix something!! :D :D.
        let gear_d: u16 = start + (RESISTANCE_PER_GEAR * 1); // Button::D
        let gear_n: u16 = start + (RESISTANCE_PER_GEAR * 2); // Button::N
        let gear_r: u16 = start + (RESISTANCE_PER_GEAR * 3); // Button::R
        let gear_p: u16 = start + (RESISTANCE_PER_GEAR * 4); // Button::P

        // Figure out what gear is in by the actuator potentiometer value.
        // The actual GEAR position (which is what we need!), depends on where the 'P' gear position is.
        // Can't use `match`, because `runtime values cannot be referenced in patterns`!!
        let gear: u16 = (pot - start).try_into().unwrap();
        trace!(
            "find_gear(): gear={}; gear_p={}; gear_r={}; gear_n={}; gear_d={}",
            gear,
            gear_p,
            gear_r,
            gear_n,
            gear_d
        );
        if (gear > (gear_p - ALLOWED_GEAR_DIFFERENCE_RESISTANCE))
            && (gear < (gear_p + ALLOWED_GEAR_DIFFERENCE_RESISTANCE))
        {
            0 // Button::P
        } else if (gear > (gear_r - ALLOWED_GEAR_DIFFERENCE_RESISTANCE))
            && (gear < (gear_r + ALLOWED_GEAR_DIFFERENCE_RESISTANCE))
        {
            1 // Button::R
        } else if (gear > (gear_n - ALLOWED_GEAR_DIFFERENCE_RESISTANCE))
            && (gear < (gear_n + ALLOWED_GEAR_DIFFERENCE_RESISTANCE))
        {
            2 // Button::N
        } else if (gear > (gear_d - ALLOWED_GEAR_DIFFERENCE_RESISTANCE))
            && (gear < (gear_d + ALLOWED_GEAR_DIFFERENCE_RESISTANCE))
        {
            3 // Button::D
        } else {
            4 // Button::UNSET
        }
    }

    // =====
    // NOTE: These should really be private functions, but I need them to do the calibration.

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

        return self.verify_moved(actuator_pot_1, actuator_pot_2).await;
    }

    // Check if after-move position is between pos1 max/min.
    // Returns:
    //   * TRUE  => Have moved.
    //   * FALSE => Have not moved.
    async fn verify_moved(&mut self, before: u16, after: u16) -> bool {
        // NOTE: We only check that it HAVE moved, not with how much.
        // NOTE: Take a +-10Ω difference on the reading. The ADC in the Pico isn't very accurate.
        trace!("verify_moved(before={}, after={})", before, after);
        if before < 25 {
            debug!(
                "Can't verify move, no resonable value from pot - return 'FALSE' (Have NOT moved)"
            );
            return false;
        }

        let before_min = before - ALLOWED_GEAR_DIFFERENCE_RESISTANCE;
        let before_max = before + ALLOWED_GEAR_DIFFERENCE_RESISTANCE;
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

    // Read the actuator potentiometer value.
    // NOTE: Should probably get an average here.
    //       https://rust-classes.com/chapter_embedded_pi_input
    //       However, that takes forever! There's a really annoying delay before something
    //       happens, so just take this value. However, it turns out it's within +/- 10Ω.
    //       We can deal with that elsewhere..
    pub async fn read_pot(&mut self) -> u16 {
        return self.adc.read(&mut self.feedback).await.unwrap();
    }
}
