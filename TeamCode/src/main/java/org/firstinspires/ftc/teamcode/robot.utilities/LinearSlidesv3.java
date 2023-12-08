package org.firstinspires.ftc.teamcode.robot.utilities;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearSlidesv3 {
    private DcMotor leftSlide;
    private DcMotor rightSlide;

    private final int liftLevelZero = 0;
    private final int liftLowPosition = 1500;
    private final int liftMidPosition = 2000;
    private final int liftHighPosition = 2300;

    public enum SlideState {
        LEVEL_ZERO,
        LOW_POSITION,
        MID_POSITION,
        HIGH_POSITION,
        IDLE
    }

    private SlideState currentState;

    public LinearSlidesv3(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotor.class, "leftlift");
        rightSlide = hardwareMap.get(DcMotor.class, "rightlift");

        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Run without encoder for follower

        currentState = SlideState.IDLE;
    }

    public void setCurrentState(SlideState state) {
        currentState = state;
    }

    public SlideState getCurrentState() {
        return currentState;
    }

    public void runStateMachine() {
        int targetPosition = 0;

        switch (currentState) {
            case LEVEL_ZERO:
                targetPosition = liftLevelZero;
                moveToPosition(targetPosition);
                break;
            case LOW_POSITION:
                targetPosition = liftLowPosition;
                moveToPosition(targetPosition);
                break;
            case MID_POSITION:
                targetPosition = liftMidPosition;
                moveToPosition(targetPosition);
                break;
            case HIGH_POSITION:
                targetPosition = liftHighPosition;
                moveToPosition(targetPosition);
                break;
            case IDLE:
                stopSlides();
                //targetPosition = 0;
                //moveToPosition(targetPosition);
                break;
            default:
                stopSlides();
                break;
        }

    }

    private void moveToPosition(int targetPosition) {
        leftSlide.setTargetPosition(targetPosition);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(0.5);
        //rightSlide.setPower(1.0);
        // The right slide follows the left slide without encoder input
        while (leftSlide.isBusy()) {
            // Continuously update the power of the right slide to match the left slide
            rightSlide.setPower(leftSlide.getPower());
        }
        

        stopSlides();
    }
    public void stopSlides() {
        leftSlide.setPower(0);
        rightSlide.setPower(0);
    }

    public void resetEncoders() {
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getLeftLiftPosition() {
        return leftSlide.getCurrentPosition();
    }

    public int getRightLiftPosition() {
        return rightSlide.getCurrentPosition();
    }
}
