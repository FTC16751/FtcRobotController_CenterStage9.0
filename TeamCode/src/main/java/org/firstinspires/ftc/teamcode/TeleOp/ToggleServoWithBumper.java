package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ToggleServoWithBumper", group = "coach")
public class ToggleServoWithBumper extends LinearOpMode {

    private Servo servoClaw;
    private boolean clawClosed = true; // Initial state of the claw

    @Override
    public void runOpMode() {
        servoClaw = hardwareMap.servo.get("servo_claw"); // Replace "servo_claw" with your servo's name

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.left_bumper) {
                // Toggle the claw state when the left bumper is pressed
                if (clawClosed) {
                    openClaw();
                } else {
                    closeClaw();
                }

                // Wait to prevent continuous toggling while the bumper is held
                while (gamepad2.left_bumper) {
                    // Do nothing until the bumper is released
                }
            }

            // Other TeleOp controls or logic can be added here

            telemetry.update();
            idle();
        }
    }

    private void openClaw() {
        // Set servo position to open the claw
        servoClaw.setPosition(0.5); // Adjust this value based on your servo's range
        clawClosed = false;
    }

    private void closeClaw() {
        // Set servo position to close the claw
        servoClaw.setPosition(0.0); // Adjust this value based on your servo's range
        clawClosed = true;
    }
}
