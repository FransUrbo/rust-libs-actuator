#![no_std]

use defmt::{error, info, trace};

use embassy_rp::adc;
use embassy_rp::adc::{
    Adc, AdcPin, Async as AdcAsync, Config as AdcConfig, InterruptHandler as ADCInterruptHandler,
};
use embassy_rp::gpio::{AnyPin, Drive, Level, Output, Pull, SlewRate};
use embassy_rp::interrupt::typelevel::{Binding, ADC_IRQ_FIFO};
use embassy_rp::peripherals::ADC;
use embassy_rp::Peripheral;
use embassy_time::Timer;

use core::marker::PhantomData;

// =====
// Setup static values, measured on the actuator.

// The resistance on the actuator at MIN and MAX throw is 550Ω and 4690Ω respectively.
// NOTE: The manufacturer say: 'Actual resistance value may vary within the 0-10kΩ range based on stroke length'.
//       I have a 4"/35lbs throw actuator: https://www.progressiveautomations.com/products/linear-actuator-with-potentiometer?variant=18277322391619.
static RESISTANCE_MIN_THROW: i16 = 550; // Ω
static RESISTANCE_MAX_THROW: i16 = 4690; // Ω

// The total throw from MIN to MAX is `100.60mm` (`3.9605"`).
static TOTAL_THROW_DISTANCE: f32 = 100.60; // mm

// Moving the actuator between endpoints takes exactly (!) 7s@4V; 2s@12V.
static TOTAL_THROW_TIME: i16 = 2000; // ms

// The distance between 'P' and 'D' is 70mm.
static DISTANCE_BETWEEN_POSITIONS: f32 = (70 / 3) as f32; // mm

// The allowed +/- value between MIN and MAX gear position is 2mm.
static ALLOWED_GEAR_DIFFERENCE: u8 = 2; // mm

// =====
// These are the actual information we're interested in.
static TOTAL_THROW_RESISTANCE: i16 = RESISTANCE_MAX_THROW - RESISTANCE_MIN_THROW;
static THROW_RESISTANCE_PER_1MM: i16 = TOTAL_THROW_RESISTANCE / TOTAL_THROW_DISTANCE as i16;
static POSITION_MATCH_RANGE: i16 = THROW_RESISTANCE_PER_1MM * ALLOWED_GEAR_DIFFERENCE as i16;

static RESISTANCE_BETWEEN_POSITIONS: i16 =
    THROW_RESISTANCE_PER_1MM * DISTANCE_BETWEEN_POSITIONS as i16;

static THROW_TIME_PER_1MM: f32 = (TOTAL_THROW_TIME as f32 / TOTAL_THROW_DISTANCE as f32); // * 1000.0;
static THROW_TIME_PER_GEAR: u64 = (THROW_TIME_PER_1MM * DISTANCE_BETWEEN_POSITIONS as f32) as u64;

// =====

pub struct Actuator<'l, A: AdcPin> {
    motor_plus: Output<'l>,
    motor_minus: Output<'l>,
    feedback: adc::Channel<'l>,
    adc: Adc<'l, AdcAsync>,
    phantom: PhantomData<&'l A>,
}

impl<'l, A: AdcPin> Actuator<'l, A> {
    pub fn new(
        pin_motor_plus: AnyPin,
        pin_motor_minus: AnyPin,
        pot_pin: A,
        adc: impl Peripheral<P = ADC> + 'l,
        irqs: impl Binding<ADC_IRQ_FIFO, ADCInterruptHandler>,
    ) -> Self {
        // Initialize the motor pins.
        let mut actuator_motor_plus = Output::new(pin_motor_plus, Level::Low); // Actuator/Motor Relay (#1)
        let mut actuator_motor_minus = Output::new(pin_motor_minus, Level::Low); // Actuator/Motor Relay (#2)

        actuator_motor_plus.set_drive_strength(Drive::_4mA);
        actuator_motor_plus.set_slew_rate(SlewRate::Slow);

        actuator_motor_minus.set_drive_strength(Drive::_4mA);
        actuator_motor_minus.set_slew_rate(SlewRate::Slow);

        // Initialize the potentiometer pin.
        let adc = Adc::new(adc, irqs, AdcConfig::default());
        let actuator_potentiometer = adc::Channel::new_pin(pot_pin, Pull::None); // Actuator/Potentiometer Brush

        // => `Distance between gear positions=23.0mm; Throw time/mm=19.880716ms`.
        trace!(
            "Distance between gear positions={}mm; Throw time/mm={}ms",
            DISTANCE_BETWEEN_POSITIONS,
            THROW_TIME_PER_1MM
        );

        Self {
            motor_plus: actuator_motor_plus,
            motor_minus: actuator_motor_minus,
            feedback: actuator_potentiometer,
            adc: adc,
            phantom: PhantomData,
        }
    }

    // Move the actuator.
    pub async fn move_actuator(&mut self, gears: i8) {
        // Set both pins to LOW to brake the motor in the actuator.
        // NOTE: The Arduino example say to set them HIGH, but that will turn ON the relays! (??)
        // https://www.progressiveautomations.com/blogs/how-to/how-to-use-relays-to-control-linear-actuators
        self.motor_plus.set_low();
        self.motor_minus.set_low();

        // How long to keep the pin HIGH to move to the designated gear position.
        let sleep = THROW_TIME_PER_GEAR * ((gears as i64).abs() as u64);
        trace!(
            "Move sleep='{}ms * {}gears = {}ms'",
            THROW_TIME_PER_GEAR,
            ((gears as i64).abs() as u64),
            sleep
        );

        if gears < 0 {
            info!("Moving actuator: direction=FORWARD; gears={}", gears);

            self.motor_plus.set_high();
            Timer::after_millis(sleep).await;

            self.motor_plus.set_low();
            Timer::after_millis(50).await;
        } else {
            info!("Moving actuator: direction=BACKWARD; gears={}", gears);

            self.motor_minus.set_high();
            Timer::after_millis(sleep).await;

            self.motor_minus.set_low();
            Timer::after_millis(50).await;
        }

        // TODO: Verify with the potentiometer on the actuator that we've actually moved it to the right position.
        let _actuator_pot_measurement = self.read_pot().await;
    }

    // Test actuator control. Move it backward 1mm, then forward 1mm.
    // This should be safe to do EVEN IF (!!) we're moving (for whatever reason).
    // NOTE: Can't use `move_actuator()` here, that moves whole gears, not distances (in mm).
    pub async fn test_actuator(&mut self) -> bool {
        info!("Testing actuator control");

        // Read start position (Ω).
        let position_1: i16 = self.read_pot().await;
        trace!("Actuator test position (#1): {}", position_1);

        // Move the actuator one mm BACKWARD.
        info!(
            "Moving actuator BACKWARD 1mm (sleep={}ms)",
            THROW_TIME_PER_1MM
        );
        self.motor_plus.set_high();
        Timer::after_millis(THROW_TIME_PER_1MM as u64).await;
        self.motor_plus.set_low();

        // Read position.
        let position_2: i16 = self.read_pot().await;
        trace!("Actuator test position (#2): {}", position_2);

        // Move the actuator one mm FORWARD.
        info!(
            "Moving actuator FORWARD 1mm (sleep={}ms)",
            THROW_TIME_PER_1MM
        );
        self.motor_minus.set_high();
        Timer::after_millis(THROW_TIME_PER_1MM as u64).await;
        self.motor_minus.set_low();

        // Read end position. Should be the same as start position..
        let position_3: i16 = self.read_pot().await;
        trace!("Position (#3): {}", position_3);

        // Verify move - we only check that it HAVE moved, not with how much..
        // TODO: Although.. Because we're waiting `THROW_TIME_PER_1MM`, we should
        //       have moved 1mm. As in, the difference should be `THROW_RESISTANCE_PER_1MM`.. ?
        if (position_1 == position_3) && (position_1 != position_2) {
            true
        } else {
            false
        }
    }

    // Read the actuator potentiometer value.
    async fn read_pot(&mut self) -> i16 {
        match self.adc.read(&mut self.feedback).await {
            Ok(val) => val as i16,
            Err(e) => {
                error!("Failed to read actuator porentiometer value: {:?}", e);
                0
            }
        }
    }

    // Find the current gear.
    pub async fn find_gear(&mut self, start: i16) -> i8 {
        // Get the actuator potentiometer value - ACTUAL position of the actuator.
        // On mine, it's a value between 550Ω (fully retracted - 'P') and 4680Ω (fully extended - 'D').
        // Manufacturer say between 0-10kΩ.
        let pot = self.read_pot().await;

        // Distance positions. Ok, so this is dumb, but it's clear. And will be clear to me,
        // five years from now when/if I go back in here and needs to fix something!! :D :D.
        let gear_p: i16 = start + (RESISTANCE_BETWEEN_POSITIONS * 0); // Button::P
        let gear_r: i16 = start + (RESISTANCE_BETWEEN_POSITIONS * 1); // Button::R
        let gear_n: i16 = start + (RESISTANCE_BETWEEN_POSITIONS * 2); // Button::N
        let gear_d: i16 = start + (RESISTANCE_BETWEEN_POSITIONS * 3); // Button::D

        // Figure out what gear is in by the actuator potentiometer value.
        // The actual GEAR position (which is what we need!), depends on where the 'P' gear position is.
        // Can't use `match`, because `runtime values cannot be referenced in patterns`!!
        let gear: i16 = (pot - start).try_into().unwrap();
        if (gear > (gear_p - POSITION_MATCH_RANGE)) && (gear < (gear_p + POSITION_MATCH_RANGE)) {
            0 // Button::P
        } else if (gear > (gear_r - POSITION_MATCH_RANGE))
            && (gear < (gear_r + POSITION_MATCH_RANGE))
        {
            1 // Button::R
        } else if (gear > (gear_n - POSITION_MATCH_RANGE))
            && (gear < (gear_n + POSITION_MATCH_RANGE))
        {
            2 // Button::N
        } else if (gear > (gear_d - POSITION_MATCH_RANGE))
            && (gear < (gear_d + POSITION_MATCH_RANGE))
        {
            3 // Button::D
        } else {
            4 // Button::UNSET
        }
    }
}
