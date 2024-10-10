package org.firstinspires.ftc.teamcode.TeleOp.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot_utilities.experiments.SmoothDriveMovement;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="Test Smooth Drive Movement", group="Main")
@Disabled
public class TestSmoothDriveMovement extends LinearOpMode {
        private DcMotorEx frontLeftMotor;
        private DcMotorEx frontRightMotor;
        private DcMotorEx rearLeftMotor;
        private DcMotorEx rearRightMotor;
        private SmoothDriveMovement smoothDriveMovement;
        private ElapsedTime elapsedTime;
        private double rampRateIncrement = 0.1;

        @Override
        public void runOpMode() {
            smoothDriveMovement = new SmoothDriveMovement(hardwareMap, telemetry);
            elapsedTime = new ElapsedTime();
            smoothDriveMovement.storeFirstAngle();
            waitForStart();

            while (opModeIsActive()) {
                double leftStickX = gamepad1.left_stick_x;
                double leftStickY = -gamepad1.left_stick_y; // Reversing the value if needed
                double rightStickX = gamepad1.right_stick_x;
                double rightStickY = -gamepad1.right_stick_y; // Reversing the value if needed
                smoothDriveMovement.fielddrive(leftStickX, leftStickY, rightStickX,rightStickY);

                if (gamepad1.a) {
                    // Increase ramp rate when left bumper is pressed
                    smoothDriveMovement.setRampRate(0.01);
                } else if (gamepad1.x) {
                    // Decrease ramp rate when right bumper is pressed
                    smoothDriveMovement.setRampRate(-0.01);
                }

                telemetry.addData("Ramp Rate", smoothDriveMovement.getRampRate());
                telemetry.update();
            }
        }

        // Other utility methods can go here if needed
    }
