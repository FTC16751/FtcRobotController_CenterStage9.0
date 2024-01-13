package org.firstinspires.ftc.teamcode.TeleOp.Retire;

        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "FieldCentric", group = "tutorial")
public class FieldCentric extends LinearOpMode {
    //HardwareClass robot = new HardwareClass();

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

        // Initialize imu
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();

        while(opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // Read inverse imu heading
            double botHeading = -imu.getAngularOrientation().firstAngle;

            // Rotate gamepad input x/y by hand
            // See: https://matthew-brett.github.io/teaching/rotation_2d.html
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            double frontLeftPower = rotY + rotX + turn;
            double backLeftPower = rotY - rotX + turn;
            double frontRightPower = rotY - rotX - turn;
            double backRightPower = rotY + rotX - turn;

            // Put powers in the range of -1 to 1 only if they aren't already
            // Not checking would cause us to always drive at full speed
            if (Math.abs(frontLeftPower) > 1 || Math.abs(backLeftPower) > 1 ||
                    Math.abs(frontRightPower) > 1 || Math.abs(backRightPower) > 1) {
                // Find the largest power
                double max = 0;
                max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
                max = Math.max(Math.abs(frontRightPower), max);
                max = Math.max(Math.abs(backRightPower), max);

                // Divide everything by max (it's positive so we don't need to worry
                // about signs)
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }

//            robot.motorLeftFront.setPower(frontLeftPower);
//            robot.motorLeftRear.setPower(backLeftPower);
//            robot.motorRightFront.setPower(frontRightPower);
//            robot.motorRightRear.setPower(backRightPower);

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }
    }
}