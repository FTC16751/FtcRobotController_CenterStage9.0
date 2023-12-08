package org.firstinspires.ftc.teamcode.robot.utilities;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class LinearSlides {
    private DcMotor leftSlide;
    private DcMotor rightSlide;

    private final int liftLevelZero = 0;
    private final int liftLowPosition = 560;
    private final int liftMidPosition = 1134;
    private final int liftHighPosition = 1660;

    public enum SlideState {
        LEVEL_ZERO,
        LOW_POSITION,
        MID_POSITION,
        HIGH_POSITION,
        IDLE
    }

    private SlideState currentState;

    public LinearSlides(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotor.class, "leftlift");
        rightSlide = hardwareMap.get(DcMotor.class, "rightlift");

        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        currentState = SlideState.IDLE;
    }

    public void setCurrentState(SlideState state) {
        currentState = state;
    }

    public SlideState getCurrentState() {
        return currentState;
    }

    public void runStateMachine() {
        switch (currentState) {
            case LEVEL_ZERO:
                moveToPosition(liftLevelZero);
                stopSlides();
                currentState = SlideState.LEVEL_ZERO;
                break;
            case LOW_POSITION:
                moveToPosition(liftLowPosition);
                currentState = SlideState.LOW_POSITION;
                break;
            case MID_POSITION:
                moveToPosition(liftMidPosition);
                currentState = SlideState.MID_POSITION;
                break;
            case HIGH_POSITION:
                moveToPosition(liftHighPosition);
                currentState = SlideState.HIGH_POSITION;
                break;
            case IDLE:
                break;
            default:
                break;
        }
    }

    private void moveToPosition(int targetPosition) {
        leftSlide.setTargetPosition(targetPosition);
        rightSlide.setTargetPosition(targetPosition);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(1.0);
        rightSlide.setPower(1.0);

       while ((leftSlide.isBusy() || rightSlide.isBusy())) {
            // Waiting for both slides to reach the target position
        }

       // stopSlides();
    }

    private void stopSlides() {
        leftSlide.setPower(0);
        rightSlide.setPower(0);
        resetEncoder();
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder(){
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
