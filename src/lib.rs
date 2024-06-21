#![no_std]

use defmt::{error, info};

use embassy_rp::adc;
use embassy_rp::adc::{Adc, Async as AdcAsync};
use embassy_rp::gpio::Output;
use embassy_time::Timer;

// =====
// Setup static values, measured on the actuator.

// The resistance on the actuator at MIN and MAX throw is 550Ω and 4690Ω respectively.
static RESISTANCE_MIN_THROW: i16 = 550;
static RESISTANCE_MAX_THROW: i16 = 4690;

// The total throw from MIN to MAX is `100.60mm` (`3.9605"`).
static TOTAL_THROW_DISTANCE: f32 = 100.60;

// Moving the actuator between endpoints takes exactly (!) 7s@4-5V; 2s@12V.
static TOTAL_THROW_TIME: u8 = 2;

// The distance between the different gears is 70mm.
static DISTANCE_BETWEEN_POSITIONS: u8 = 70;

// The allowed +/- value between MIN and MAX gear position is 2mm.
static ALLOWED_GEAR_DIFFERENCE: u8 = 2;

// =====
// These are the actual information we're interested in.
static TOTAL_THROW_RESISTANCE: i16 = RESISTANCE_MAX_THROW - RESISTANCE_MIN_THROW;
static THROW_RESISTANCE_PER_1MM: i16 = TOTAL_THROW_RESISTANCE / TOTAL_THROW_DISTANCE as i16;
static POSITION_MATCH_RANGE: i16 = THROW_RESISTANCE_PER_1MM * ALLOWED_GEAR_DIFFERENCE as i16;
static THROW_TIME_PER_1MM: f32 = TOTAL_THROW_TIME as f32 / TOTAL_THROW_DISTANCE as f32;

static RESISTANCE_BETWEEN_POSITIONS: i16 =
    THROW_RESISTANCE_PER_1MM * DISTANCE_BETWEEN_POSITIONS as i16;
static TIME_DISTANCE_PER_GEAR: u64 =
    (THROW_TIME_PER_1MM * DISTANCE_BETWEEN_POSITIONS as f32) as u64;

// =====

// Move the actuator.
pub async fn move_actuator(
    gears: i8, // Number of gear position(s) in either direction.
    pin_motor_plus: &mut Output<'static>,
    pin_motor_minus: &mut Output<'static>,
    pot_adc: &mut Adc<'static, AdcAsync>,
    pot_pin: &mut adc::Channel<'static>,
) {
    // Set both pins to LOW to brake the motor in the actuator.
    // NOTE: The Arduino example say to set them HIGH, but that will turn ON the relays! (??)
    // https://www.progressiveautomations.com/blogs/how-to/how-to-use-relays-to-control-linear-actuators
    pin_motor_plus.set_low();
    pin_motor_minus.set_low();

    if gears < 0 {
        info!("Moving actuator:  direction=FORWARD; gears={}", gears);

        pin_motor_plus.set_high();
        Timer::after_millis(TIME_DISTANCE_PER_GEAR * ((gears as i64).abs() as u64)).await;

        pin_motor_plus.set_low();
        Timer::after_millis(50).await;
    } else {
        info!("Moving actuator: direction=BACKWARD; gears={}", gears);

        pin_motor_minus.set_high();
        Timer::after_millis(TIME_DISTANCE_PER_GEAR * gears as u64).await;

        pin_motor_minus.set_low();
        Timer::after_millis(50).await;
    }

    // TODO: Verify with the potentiometer on the actuator that we've actually moved it to the right position.
    let _actuator_pot_measurement = read_pot(pot_adc, pot_pin).await;
}

// Test actuator control. Move it backward 1mm, then forward 1mm.
// This should be safe to do EVEN IF (!!) we're moving (for whatever reason).
pub async fn test_actuator(
    pin_motor_plus: &mut Output<'static>,
    pin_motor_minus: &mut Output<'static>,
    pot_adc: &mut Adc<'static, AdcAsync>,
    pot_pin: &mut adc::Channel<'static>,
) -> bool {
    info!("Testing actuator control");

    let position_1: i16 = read_pot(pot_adc, pot_pin).await;
    move_actuator(-1, pin_motor_plus, pin_motor_minus, pot_adc, pot_pin).await;

    let position_2: i16 = read_pot(pot_adc, pot_pin).await;
    Timer::after_millis(100).await;

    move_actuator(1, pin_motor_plus, pin_motor_minus, pot_adc, pot_pin).await;
    let position_3: i16 = read_pot(pot_adc, pot_pin).await;

    // Verify move - we only check that it HAVE moved, not with how much..
    if (position_1 == position_3) && (position_1 != position_2) {
        true
    } else {
        false
    }
}

// Read the actuator potentiometer value.
pub async fn read_pot(
    pot_adc: &mut Adc<'static, AdcAsync>,
    pot_pin: &mut adc::Channel<'static>,
) -> i16 {
    match pot_adc.read(pot_pin).await {
        Ok(val) => val as i16,
        Err(e) => {
            error!("Failed to read actuator porentiometer value: {:?}", e);
            0
        }
    }
}

// Find the current gear.
pub async fn find_gear(
    pot_adc: &mut Adc<'static, AdcAsync>,
    pot_pin: &mut adc::Channel<'static>,
    start: i16,
) -> i8 {
    // Get the actuator potentiometer value - ACTUAL position of the actuator.
    // On mine, it's a value between 550Ω (fully retracted - 'P') and 4680Ω (fully extended - 'D').
    // Manufacturer say between 0-10kΩ.
    let pot = read_pot(pot_adc, pot_pin).await;

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
    } else if (gear > (gear_r - POSITION_MATCH_RANGE)) && (gear < (gear_r + POSITION_MATCH_RANGE)) {
        1 // Button::R
    } else if (gear > (gear_n - POSITION_MATCH_RANGE)) && (gear < (gear_n + POSITION_MATCH_RANGE)) {
        2 // Button::N
    } else if (gear > (gear_d - POSITION_MATCH_RANGE)) && (gear < (gear_d + POSITION_MATCH_RANGE)) {
        3 // Button::D
    } else {
        4 // Button::UNSET
    }
}
