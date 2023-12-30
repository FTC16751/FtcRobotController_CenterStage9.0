package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Mecanum Rotation with Choice", group = "coach")
public class MecanumRotation extends LinearOpMode {

    private static final double PI = Math.PI;
    private static final int ENCODER_RESOLUTION = 527;
    private static final double WHEEL_DIAMETER = 3.779; // inches
    private static final double ROBOT_DIAGONAL = 24; // inches

    private DcMotor frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;
    private ElapsedTime runtime = new ElapsedTime();

    private final double[] ROTATION_ANGLES = {45.0, 90.0, 135.0, 180.0}; // Button mappings to angles

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Initializing...", "");
        telemetry.update();

        // Initialize and configure motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        rearLeftMotor = hardwareMap.dcMotor.get("rearLeftMotor");
        rearRightMotor = hardwareMap.dcMotor.get("rearRightMotor");

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        double targetAngle = 0.0;
        // Display available angle choices
        telemetry.addData("Select Rotation:", "");
        telemetry.update();
        if (gamepad1.x) {
            targetAngle = 45;
        }
        else if (gamepad1.y) {
            targetAngle = 90;
        }
        else if (gamepad1.b) {
            targetAngle = 135;
        }
        else if (gamepad1.x) {
            targetAngle = 180;
        }


        // Calculate motor rotations and target encoder counts
        double targetRadians = Math.toRadians(targetAngle);
        double wheelRotations = targetRadians * ROBOT_DIAGONAL / calculateWheelCircumference();
        double countsPerRotation = calculateCountsPerRotation();
        double targetCounts = wheelRotations * countsPerRotation;

        // Set motor directions based on your robot configuration
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Run motors to target encoder counts
        frontLeftMotor.setTargetPosition((int) targetCounts);
        frontRightMotor.setTargetPosition((int) targetCounts);
        rearLeftMotor.setTargetPosition((int) targetCounts);
        rearRightMotor.setTargetPosition((int) targetCounts);

        frontLeftMotor.setPower(0.5); // Adjust power as needed
        frontRightMotor.setPower(0.5);
        rearLeftMotor.setPower(0.5);
        rearRightMotor.setPower(0.5);

        // Show telemetry options during rotation
        boolean showPositionOnly;
        while (opModeIsActive() && (
                frontLeftMotor.isBusy() || frontRightMotor.isBusy() ||
                        rearLeftMotor.isBusy() || rearRightMotor.isBusy())) {


            double elapsedTime = runtime.seconds();
            double estimatedTime = (targetCounts - Math.min(
                    frontLeftMotor.getCurrentPosition(),
                    frontRightMotor.getCurrentPosition()
            )) / countsPerRotation;

            telemetry.addData("Target Position:", targetCounts);
            telemetry.addData("Front Left:", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right:", frontRightMotor.getCurrentPosition());
            telemetry.addData("Rear Left:", rearLeftMotor.getCurrentPosition());
            telemetry.addData("Rear Right:", rearRightMotor.getCurrentPosition());
            telemetry.addData("Elapsed Time:", elapsedTime);
            telemetry.addData("Estimated Time:", estimatedTime);
        }

        telemetry.update();
        sleep(50); // Avoid flooding telemetry

        // Stop motors and reset encoders
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        rearLeftMotor.setPower(0.0);
        rearRightMotor.setPower(0.0);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Rotation Complete!", "");
        telemetry.update();
        sleep(1000); // Allow time for user to see completion message
    }

    private double calculateWheelCircumference() {
        return PI * WHEEL_DIAMETER;
    }
    private double calculateCountsPerRotation() {
        return ENCODER_RESOLUTION / 4.0; // 4 encoders per wheel
    }
// Add any additional post-rotation actions here (e.g., move forward, etc.)
}