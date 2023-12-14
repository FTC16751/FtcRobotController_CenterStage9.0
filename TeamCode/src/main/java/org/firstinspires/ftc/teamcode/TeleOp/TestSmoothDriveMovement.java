package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.utilities.SmoothDriveMovement;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="Test Smooth Drive Movement", group="Main")
public class TestSmoothDriveMovement extends LinearOpMode {
        private DcMotorEx frontLeftMotor;
        private DcMotorEx frontRightMotor;
        private DcMotorEx rearLeftMotor;
        private DcMotorEx rearRightMotor;
        private SmoothDriveMovement driveMovement;
        private ElapsedTime elapsedTime;
        private double rampRateIncrement = 0.1;

        @Override
        public void runOpMode() {
            driveMovement = new SmoothDriveMovement(hardwareMap, telemetry);
            elapsedTime = new ElapsedTime();

            waitForStart();

            while (opModeIsActive()) {
                /*double forwardReverse = -gamepad1.left_stick_y; // Reversing the value if needed
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                driveMovement.drive(forwardReverse, strafe, turn);
*/

                double leftStickX = gamepad1.left_stick_x;
                double leftStickY = -gamepad1.left_stick_y; // Reversing the value if needed
                double rightStickX = gamepad1.right_stick_x;
                double rightStickY = -gamepad1.right_stick_y; // Reversing the value if needed

                if(gamepad1.left_bumper) {
                    driveMovement.drive(leftStickX, leftStickY, rightStickX, rightStickY,true);
                } else {
                    driveMovement.drive(leftStickX, leftStickY, rightStickX, rightStickY, false);
                }

                if (gamepad1.left_bumper) {
                    // Increase ramp rate when left bumper is pressed
                    driveMovement.setRampRate(0.1);
                } else if (gamepad1.right_bumper) {
                    // Decrease ramp rate when right bumper is pressed
                    driveMovement.setRampRate(-0.1);
                }

                //telemetry.addData("Ramp Rate", driveMovement.getRampRate());
                telemetry.update();
            }
        }

        // Other utility methods can go here if needed
    }
