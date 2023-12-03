package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DualLinearSlides extends LinearOpMode {

    private DcMotor leftSlide;
    private DcMotor rightSlide;

    // Positions for the linear slides
    private final int liftLevelZero = 0;
    private final int liftLowPosition = 560;
    private final int liftMidPosition = 1134;
    private final int liftHighPosition = 1660;

    @Override
    public void runOpMode() {
        initializeHardware();

        waitForStart();

        // Move both slides to different positions in sync
        moveToPosition(liftLevelZero);
        sleep(1000); // Delay for demonstration purposes
        moveToPosition(liftLowPosition);
        sleep(1000);
        moveToPosition(liftMidPosition);
        sleep(1000);
        moveToPosition(liftHighPosition);
        sleep(1000);

        // Stop the slides after movement
        stopSlides();
    }

    private void initializeHardware() {
        //HardwareMap hardwareMap = hardwareMap;
        leftSlide = hardwareMap.get(DcMotor.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotor.class, "right_slide");

        // Set motor directions and zero power behavior
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders and run using encoder
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void moveToPosition(int targetPosition) {
        // Set the target position for both slides and run to that position
        leftSlide.setTargetPosition(targetPosition);
        rightSlide.setTargetPosition(targetPosition);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(1.0);
        rightSlide.setPower(1.0);

        // Wait until both motors reach the target position
        while ((leftSlide.isBusy() || rightSlide.isBusy()) && opModeIsActive()) {
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Left Position", leftSlide.getCurrentPosition());
            telemetry.addData("Right Position", rightSlide.getCurrentPosition());
            telemetry.update();
        }

        // Stop both motors after reaching the target position
        stopSlides();
    }

    private void stopSlides() {
        leftSlide.setPower(0);
        rightSlide.setPower(0);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
