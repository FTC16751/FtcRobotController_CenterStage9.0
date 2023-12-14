package org.firstinspires.ftc.teamcode.TeleOp;





import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.utilities.NewDriveUtil2024;

@TeleOp(name="Simple TeleOp", group="Main")
public class SimpleTeleOp extends LinearOpMode {

    private NewDriveUtil2024 driveUtil;
    private final double DEADZONE = 0.1; // Adjust deadzone threshold
    private final double MAX_MOTOR_SPEED = 1.0; // Adjust maximum motor power

    @Override
    public void runOpMode() throws InterruptedException {
        int driveMode = 1;

        // Initialize DriveUtil
        driveUtil = new NewDriveUtil2024(hardwareMap);

        // Wait for start
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.start) driveMode = 1;
            if (gamepad1.back) driveMode = 2;
            // Get and process joystick inputs
            double forwardPower = -gamepad1.left_stick_y; // Forward/backward
            double strafePower = gamepad1.left_stick_x; // Left/right strafe
            double turnPower = gamepad1.right_stick_x; // Turn

            //call the respective drive mode
            if (driveMode == 1) {
                driveUtil.setMecanumPower(forwardPower, strafePower, turnPower);
                telemetry.addData("Drive Mode", "Arcade");
            }
            else if (driveMode == 2) {
                driveUtil.fieldDrive(forwardPower, strafePower, turnPower);
                telemetry.addData("Drive Mode", "field");
            }
            telemetry.addData("ramp rate: ", driveUtil.getRampRate());
            telemetry.addData("left front power: ", driveUtil.getmotorPower(driveUtil.frontLeftMotor));
            telemetry.addData("left front power: ", driveUtil.getmotorPower(driveUtil.frontRightMotor));
            telemetry.addData("left front power: ", driveUtil.getmotorPower(driveUtil.rearLeftMotor));
            telemetry.addData("left front power: ", driveUtil.getmotorPower(driveUtil.rearRightMotor));
            telemetry.update();
        }
    }
}
