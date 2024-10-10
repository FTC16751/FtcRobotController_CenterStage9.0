package org.firstinspires.ftc.teamcode.TeleOp.Experiments;





import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot_utilities.NewDriveUtil2024;

@TeleOp(name="Test NewDriveUtil PID TeleOp", group="Main")
@Disabled
public class SimpleTeleOp_NewDriveUtil2024 extends LinearOpMode {

    private final NewDriveUtil2024 driveUtil = new NewDriveUtil2024(this);
    private final double DEADZONE = 0.1; // Adjust deadzone threshold
    private final double MAX_MOTOR_SPEED = 1.0; // Adjust maximum motor power
    double DRIVE_SPEED = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        int driveMode = 1;

        // Initialize DriveUtil
        driveUtil.init(hardwareMap,telemetry);

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
                driveUtil.driveMecanum(forwardPower, strafePower, turnPower,gamepad1.right_stick_y, DRIVE_SPEED);
                //driveUtil.setMecanumPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, DRIVE_SPEED);
                telemetry.addData("Drive Mode", "Arcade");
            }
            else if (driveMode == 2) {
                driveUtil.driveFieldOriented(forwardPower, strafePower, turnPower,gamepad1.right_stick_y, DRIVE_SPEED);
                telemetry.addData("Drive Mode", "field");
            }

            if (gamepad2.dpad_up){
                driveUtil.moveForwardPID(24,.30,30000);
            }
            if (gamepad2.dpad_down){
                driveUtil.moveBackwardPID(24,.30,30000);
            }
            if (gamepad2.dpad_left) {
                driveUtil.strafeLeftPID(24,.25,30000);
            }
            if (gamepad2.dpad_right) {
                driveUtil.strafeRightPID(24,.25,30000);
            }
            if (gamepad1.left_bumper) {
                // rotate left for 90 degrees
                driveUtil.turnPID(true,90,.30,30000);
            }
            if (gamepad1.right_bumper) {
                // rotate left for 90 degrees
                driveUtil.turnPID(false,90,.30,30000);
            }
            if (gamepad2.y) {
                driveUtil.movePID(NewDriveUtil2024.Direction.FORWARD,25, .30, 30000);
            }
            if (gamepad2.a) {
                driveUtil.movePID(NewDriveUtil2024.Direction.BACKWARD,25, .30, 30000);
            }
            if (gamepad2.x) {
                driveUtil.movePID(NewDriveUtil2024.Direction.LEFT,25, .30, 30000);
            }
            if (gamepad2.b) {
                driveUtil.movePID(NewDriveUtil2024.Direction.RIGHT,25, .30, 30000);
            }
            telemetry.addLine("PID Testing Controls | Gamepad2:")
                    .addData("dpad up", "move forward")
                    .addData("dpad down", "move backwards")
                    .addData("dpad left", "strafe left")
                    .addData("dpad right", "strafe right")
                    .addData("left bumper", "rotate left 90 degrees")
                    .addData("right bumper", "rotate right 90 degrees");
            telemetry.addLine("right joystick | ")
                    .addData("x", gamepad1.right_stick_x)
                    .addData("y", gamepad1.right_stick_y);
            telemetry.addData("left front encoder distance: ", driveUtil.getEncoderDistance(driveUtil.left_front_motor));
            telemetry.addData("left front power: ", driveUtil.getmotorPower(driveUtil.left_front_motor));
            telemetry.addData("left front power: ", driveUtil.getmotorPower(driveUtil.right_front_motor));
            telemetry.addData("left front power: ", driveUtil.getmotorPower(driveUtil.left_rear_motor));
            telemetry.addData("left front power: ", driveUtil.getmotorPower(driveUtil.right_rear_motor));
            telemetry.update();
        }
    }
}
