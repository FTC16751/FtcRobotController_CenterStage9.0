package org.firstinspires.ftc.teamcode.robot_utilities.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot_utilities.NewDriveUtil2024;

@Autonomous(name = "TestAutonomousNewUtil2024", group = "Tests")
public class TestAutonomousNewUtil2024 extends LinearOpMode {
    NewDriveUtil2024 driveUtil = new NewDriveUtil2024(this);

    @Override
    public void runOpMode() {
        driveUtil.init(hardwareMap,telemetry); // Initialize motors using utility class

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        double targetSpeed = 0.5; // Change this speed as needed

        // Drive forward for 2 seconds
        telemetry.addData("Action", "Driving forward");
        telemetry.addData("Speed", targetSpeed);
        telemetry.update();

        driveUtil.driveRobotForward(targetSpeed);
        sleep(2000); // Adjust this time as needed

        telemetry.addData("Action", "Stopping");
        telemetry.update();

        driveUtil.stopRobot();
        sleep(1000); // Wait for 1 second before moving backward

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
}
