package org.firstinspires.ftc.teamcode.robot_utilities.experiments;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class LinearSlidesSimple {
    private DcMotor leftSlide;
    private DcMotor rightSlide;

    // Positions for the linear slides
    private final int liftLevelZero = 0;
    private final int liftLowPosition = 560;
    private final int liftMidPosition = 1134;
    private final int liftHighPosition = 1660;

    public LinearSlidesSimple(HardwareMap hardwareMap) {
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

    public void moveToPosition(int targetPosition) {
        leftSlide.setTargetPosition(targetPosition);
        rightSlide.setTargetPosition(targetPosition);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(1.0);
        rightSlide.setPower(1.0);

        while ((leftSlide.isBusy() || rightSlide.isBusy())) {
            // Waiting for both slides to reach the target position
        }

        stopSlides();
    }

    public void stopSlides() {
        leftSlide.setPower(0);
        rightSlide.setPower(0);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getLiftLevelZero() {
        return liftLevelZero;
    }

    public int getLiftLowPosition() {
        return liftLowPosition;
    }

    public int getLiftMidPosition() {
        return liftMidPosition;
    }

    public int getLiftHighPosition() {
        return liftHighPosition;
    }
}
