package org.firstinspires.ftc.teamcode.TeleOp.Experiments;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;
@TeleOp(name = "FieldCentric4", group = "tutorial")
public class FieldCentric4 extends LinearOpMode {
    //HardwareClass robot = new HardwareClass();
// The IMU sensor object
    IMU imu;
    // Create an object to receive the IMU angles
    YawPitchRollAngles robotOrientation;
    YawPitchRollAngles lastOrientation;
    @Override public void runOpMode() {
        // Declare our motors
        // robot.init(hardwareMap);

        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft  = hardwareMap.dcMotor.get("Front_Left");
        DcMotor motorBackLeft   = hardwareMap.dcMotor.get("Rear_Left");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        DcMotor motorBackRight  = hardwareMap.dcMotor.get("Rear_Right");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);

        // Initialize imu
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        robotOrientation = imu.getRobotYawPitchRollAngles();


        waitForStart();

        while(opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            if (gamepadRateLimit.hasExpired() && gamepad1.a) {
                imu.resetYaw();
                gamepadRateLimit.reset();
            }

            // Read inverse imu heading
            double botHeading   = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // Get current orientation
            YawPitchRollAngles currentOrientation = imu.getRobotYawPitchRollAngles();

            // Calculate heading change since last loop iteration
            double deltaHeading = currentOrientation.getYaw(AngleUnit.RADIANS) - robotOrientation.getYaw(AngleUnit.RADIANS);

            // Update last orientation
            lastOrientation = currentOrientation;

            telemetry.addData("Bot Heading: ", currentOrientation.getYaw(AngleUnit.RADIANS));
            telemetry.addData("last Heading reading: ", robotOrientation.getYaw(AngleUnit.RADIANS));
            telemetry.addData("last Heading reading: ", lastOrientation.getYaw(AngleUnit.RADIANS));

            // Rotate gamepad input x/y by hand
            // See: https://matthew-brett.github.io/teaching/rotation_2d.html
            double rotX1 = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY1 = x * Math.sin(botHeading) + y * Math.cos(botHeading);
            // Adjust joystick inputs based on heading change
            double rotX = x * Math.cos(deltaHeading) - y * Math.sin(deltaHeading);
            double rotY = x * Math.sin(deltaHeading) + y * Math.cos(deltaHeading);

            double frontLeftPower = rotY + rotX + turn;
            double backLeftPower = rotY - rotX + turn;
            double frontRightPower = rotY - rotX - turn;
            double backRightPower = rotY + rotX - turn;

            // Normalize motor powers
            double max = Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                    Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
            );

            // Put powers in the range of -1 to 1 only if they aren't already
            // Not checking would cause us to always drive at full speed
            if (max > 1) {
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);


            telemetry.addData("rotX1: ", rotX1);
            telemetry.addData("rotX: ", rotX);
            telemetry.addData("rotY1: ",rotY1);
            telemetry.addData("rotY: ", rotY);
            telemetry.addData("botHeading new: ", botHeading);

            // Retrieve Rotational Angles and Velocities
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.RADIANS));

            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.update();
        }
    }
}