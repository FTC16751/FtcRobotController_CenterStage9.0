package org.firstinspires.ftc.teamcode.robot_utilities.tests;

// Import necessary libraries
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot_utilities.NewDriveUtil2024;

// Ensure the appropriate annotations and class declaration for the TeleOp mode
@TeleOp(name = "TestTeleOpMoveLinear", group = "_ATests")
@Disabled
public class TestTeleOpMove extends LinearOpMode {
    // Initialize an instance of NewDriveUtil2024
    NewDriveUtil2024 driveUtil = new NewDriveUtil2024(this);

    @Override
    public void runOpMode() {
        // Initialization logic
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize hardware components or any necessary configurations
        // (if needed for your specific robot)
        driveUtil.init(hardwareMap,telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get gamepad joystick values for movement
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double yaw = gamepad1.right_stick_x;

            // Call the moveRobot method from NewDriveUtil2024 with gamepad joystick values
            driveUtil.moveRobot(x, y, yaw);


            double targetSpeed = 0.25; // Change this speed as needed
            if(gamepad1.y) {
                // Drive forward for 2 seconds
                telemetry.addData("Action", "Driving forward");
                telemetry.addData("Speed", targetSpeed);
                telemetry.update();

                driveUtil.driveRobotForward(targetSpeed);
                sleep(2000); // Adjust this time as needed

                telemetry.addData("Action", "Stopping");
                telemetry.update();

                driveUtil.stopRobot();

            }
            else if (gamepad1.a) {
                // Drive backward for 2 seconds
                telemetry.addData("Action", "Driving backward");
                telemetry.addData("Speed", targetSpeed);
                telemetry.update();

                driveUtil.driveRobotBackward(targetSpeed);
                sleep(2000); // Adjust this time as needed

                telemetry.addData("Action", "Stopping");
                telemetry.update();

                driveUtil.stopRobot();
            }
            else if (gamepad1.x) {
                // Drive left for 2 seconds
                telemetry.addData("Action", "Driving left");
                telemetry.addData("Speed", targetSpeed);
                telemetry.update();

                driveUtil.driveRobotStrafeLeft(targetSpeed);
                sleep(2000); // Adjust this time as needed

                telemetry.addData("Action", "Stopping");
                telemetry.update();

                driveUtil.stopRobot();
            }
            else if (gamepad1.b) {
                // Drive right for 2 seconds
                telemetry.addData("Action", "Driving left");
                telemetry.addData("Speed", targetSpeed);
                telemetry.update();

                driveUtil.driveRobotStrafeRight(targetSpeed);
                sleep(2000); // Adjust this time as needed

                telemetry.addData("Action", "Stopping");
                telemetry.update();

                driveUtil.stopRobot();
            }
            else if (gamepad1.left_bumper) {

                telemetry.addData("Action", "rotate left 90");
                telemetry.addData("Speed", targetSpeed);
                telemetry.update();

                driveUtil.rotateLeft90Degrees();

                telemetry.addData("Action", "Stopping");
                telemetry.update();
            }
            else if (gamepad1.right_bumper) {
                // rotate right for 90 degrees
                telemetry.addData("Action", "rotateright 90");
                telemetry.addData("Speed", targetSpeed);
                telemetry.update();

                driveUtil.rotateRight90Degrees();

                telemetry.addData("Action", "Stopping");
                telemetry.update();
            }
            else if (gamepad1.left_trigger >0.5) {
                // rotate left for 45 degrees
                telemetry.addData("Action", "rotate left 45");
                telemetry.addData("Speed", targetSpeed);
                telemetry.update();

                driveUtil.rotateLeft45Degrees();

                telemetry.addData("Action", "Stopping");
                telemetry.update();
            }
            else if (gamepad1.right_trigger >0.5) {
                // rotate left for 45 degrees
                telemetry.addData("Action", "rotate right 45");
                telemetry.addData("Speed", targetSpeed);
                telemetry.update();

                driveUtil.rotateRight45Degrees();

                telemetry.addData("Action", "Stopping");
                telemetry.update();
            }
            else if (gamepad1.dpad_left) {
                // strafe left for x distance
                telemetry.addData("Action", "strafe left for 61 cm");
                telemetry.addData("Speed", targetSpeed);
                telemetry.update();

                driveUtil.driveRobotDistanceStrafeLeft(60.96, targetSpeed);

                telemetry.addData("Action", "Stopping");
                telemetry.update();
            }
            else if (gamepad1.dpad_right) {
                // strafe left for x distance
                telemetry.addData("Action", "strafe right for 61 cm");
                telemetry.addData("Speed", targetSpeed);
                telemetry.update();

                driveUtil.driveRobotDistanceStrafeRight(60.96, targetSpeed);

                telemetry.addData("Action", "Stopping");
                telemetry.update();
            }
            else if (gamepad1.dpad_up) {
                // strafe left for x distance
                telemetry.addData("Action", "move forward for 61 cm");
                telemetry.addData("Speed", targetSpeed);
                telemetry.update();

                driveUtil.driveRobotDistanceForward(60.96,targetSpeed);

                telemetry.addData("Action", "Stopping");
                telemetry.update();
            }
            else if (gamepad1.dpad_down) {
                // strafe left for x distance
                telemetry.addData("Action", "move backward for 61 cm");
                telemetry.addData("Speed", targetSpeed);
                telemetry.update();

                driveUtil.driveRobotDistanceBackward(60.96,targetSpeed);

                telemetry.addData("Action", "Stopping");
                telemetry.update();
            }
            else if (gamepad2.dpad_up) {
                // strafe left for x distance
                telemetry.addData("Action", "move forward for 24 inches");
                telemetry.addData("Speed", targetSpeed);
                telemetry.update();

                driveUtil.driveRobotDistanceForward(24,targetSpeed, NewDriveUtil2024.UnitOfMeasure.INCHES);

                telemetry.addData("Action", "Stopping");
                telemetry.update();
            }
            // Additional telemetry for testing (can be removed in final code)
            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("Yaw", yaw);
            telemetry.update();
        }
    }
}

