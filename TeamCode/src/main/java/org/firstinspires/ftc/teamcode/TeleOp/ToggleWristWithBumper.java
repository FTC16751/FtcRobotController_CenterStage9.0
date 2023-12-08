package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ToggleWristWithBumper", group = "coach")
public class ToggleWristWithBumper extends LinearOpMode {

    private Servo wristServo;
    private int wristState = 0; // 0: Down, 1: Grab, 2: Carry, 3: Score

    @Override
    public void runOpMode() {
        wristServo = hardwareMap.servo.get("wrist_servo"); // Replace "wrist_servo" with your servo's name

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.right_bumper) {
                // Increment wrist state when right bumper is pressed
                wristState = (wristState + 1) % 4;
                setWristState(wristState);

                // Wait to prevent continuous state changes while the bumper is held
                while (gamepad2.right_bumper) {
                    // Do nothing until the bumper is released
                }
            }

            // Other TeleOp controls or logic can be added here

            telemetry.update();
            idle();
        }
    }

    private void setWristState(int state) {
        // Set wrist servo position based on the state
        switch (state) {
            case 0: // Down
                wristServo.setPosition(0.0); // Adjust this value based on your servo's range
                break;
            case 1: // Grab
                wristServo.setPosition(0.3); // Adjust this value based on your servo's range
                break;
            case 2: // Carry
                wristServo.setPosition(0.6); // Adjust this value based on your servo's range
                break;
            case 3: // Score
                wristServo.setPosition(1.0); // Adjust this value based on your servo's range
                break;
            default:
                break;
        }
    }
}
