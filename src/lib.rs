#![no_std]

use defmt::{info, trace};

use embassy_rp::gpio::Output;
use embassy_time::Timer;

// Set the distance between the different mode. 70mm is the total throw from begining to end.
pub static ACTUATOR_DISTANCE_BETWEEN_POSITIONS: i8 = 70 / 3; // 3 because the start doesn't count :).

// Actually move the actuator.
// TODO: How do we actually move the actuator?
//       Is it enough to send it +3V3 or +5V on the motor relay, or does it need more power? If so,
//       we need two more MOSFETs.
pub async fn move_actuator(
    amount: i8, // Distance in mm in either direction.
    pin_motor_plus: &mut Output<'static>,
    pin_motor_minus: &mut Output<'static>,
) {
    if amount < 0 {
        info!("Moving actuator:  direction=FORWARD; amount={}", amount);
        pin_motor_minus.set_low(); // Set the other pin to low. There can be only one!

        // FAKE: Simulate move by toggling the pin HIGH and LOW `amount` (mm) times.
        let mut pos: i8 = 0; // Make sure to blink BOTH at completion of every position move.
        for i in amount..=0 {
            // FAKE: For every position, turn BOTH led on for a bit, to indicate position.
            trace!("pos={}; i={}", pos, i);
            if i % ACTUATOR_DISTANCE_BETWEEN_POSITIONS == 0 {
                if pos != 0 {
                    trace!("i % {}", ACTUATOR_DISTANCE_BETWEEN_POSITIONS);
                    pin_motor_minus.set_high();
                    pin_motor_plus.set_high();
                    Timer::after_millis(100).await;
                    pin_motor_minus.set_low();
                    pin_motor_plus.set_low();
                }

                pos += 1;
            }

            pin_motor_plus.set_high();
            Timer::after_millis(50).await;
            pin_motor_plus.set_low();
            Timer::after_millis(50).await;
        }
    } else {
        info!("Moving actuator: direction=BACKWARD; amount={}", amount);
        pin_motor_plus.set_low(); // Set the other pin to low. There can be only one!

        // FAKE: Simulate move by toggling the pin HIGH and LOW `amount` (mm) times.
        let mut pos: i8 = 0; // Make sure to blink BOTH at completion of every position move.
        for i in 0..=amount {
            // FAKE: For every position, turn BOTH led on for a bit, to indicate position.
            trace!("pos={}; i={}", pos, i);
            if i % ACTUATOR_DISTANCE_BETWEEN_POSITIONS == 0 {
                if pos != 0 {
                    trace!("i % {}", ACTUATOR_DISTANCE_BETWEEN_POSITIONS);
                    pin_motor_minus.set_high();
                    pin_motor_plus.set_high();
                    Timer::after_millis(100).await;
                    pin_motor_minus.set_low();
                    pin_motor_plus.set_low();
                }

                pos += 1;
            }

            pin_motor_minus.set_high();
            Timer::after_millis(50).await;
            pin_motor_minus.set_low();
            Timer::after_millis(50).await;
        }
    }

    // TODO: Verify with the potentiometer on the actuator that we've actually moved it to the right position.
    //       Documentation say "Actual resistance value may vary within the 0-10kΩ range based on stroke length".
}

// Test actuator control. Move it backward 1mm, then forward 1mm.
// This should be safe to do EVEN IF (!!) we're moving (for whatever reason).
pub async fn test_actuator(
    pin_motor_plus: &mut Output<'static>,
    pin_motor_minus: &mut Output<'static>,
) -> bool {
    info!("Testing actuator control");
    move_actuator(-1, pin_motor_plus, pin_motor_minus).await;
    Timer::after_millis(100).await;
    move_actuator(1, pin_motor_plus, pin_motor_minus).await;

    // TODO: How do we know the actuator test worked?

    true
}
