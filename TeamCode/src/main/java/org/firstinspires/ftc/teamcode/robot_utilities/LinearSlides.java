package org.firstinspires.ftc.teamcode.robot_utilities;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearSlides {
    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;

    private final int liftLevelZero = 0;
    private final int liftLowPosition = 1792;
    private final int liftMidPosition = 2000;
    private final int liftHighPosition = 2360;

    public enum SlideState {
        LEVEL_ZERO,
        LOW_POSITION,
        MID_POSITION,
        HIGH_POSITION,
        IDLE
    }

    private SlideState currentState;

    public LinearSlides(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftlift");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightlift");

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
                currentState = SlideState.LEVEL_ZERO;
                moveToPosition(liftLevelZero);
                stopSlides();
                break;
            case LOW_POSITION:
                currentState = SlideState.LOW_POSITION;
                moveToPosition(liftLowPosition);
                break;
            case MID_POSITION:
                currentState = SlideState.MID_POSITION;
                moveToPosition(liftMidPosition);
                break;
            case HIGH_POSITION:
                currentState = SlideState.HIGH_POSITION;
                moveToPosition(liftHighPosition);
                break;
            case IDLE:
                break;
            default:
                stopSlides();
                break;
        }
    }

    private void moveToPosition(int targetPosition) {
        leftSlide.setTargetPositionTolerance(25);
        rightSlide.setTargetPositionTolerance(25);
        leftSlide.setTargetPosition(targetPosition);
        rightSlide.setTargetPosition(targetPosition);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(.50);
        rightSlide.setPower(.50);

       while ((leftSlide.isBusy() && rightSlide.isBusy())) {
            // Waiting for both slides to reach the target position
        }
        stopSlides();
    }

    private void stopSlides() {
        leftSlide.setPower(0);
        rightSlide.setPower(0);
        //resetEncoder();
        //leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder(){
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
