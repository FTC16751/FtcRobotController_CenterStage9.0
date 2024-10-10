package org.firstinspires.ftc.team23469.robot.utilities.Production;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class LinearSlidesUtil {
    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;
    TouchSensor touchSensor;
    private final int liftLevelZero = 0;
    private final int liftLowPosition = 1792;
    private final int liftMidPosition = 2000;
    private final int liftHighPosition = 2360;

    public void decreasePosition(int decreaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = leftSlide.getCurrentPosition();
        newPosition = currentPosition - decreaseAmount;
        changePosition(newPosition);
    }
    public void increasePosition(int increaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = leftSlide.getCurrentPosition();
        newPosition = currentPosition + increaseAmount;
        changePosition(newPosition);
    }
    public void changePosition(int newPosition) {
        leftSlide.setTargetPosition(newPosition);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(1.0);
        rightSlide.setTargetPosition(newPosition);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setPower(1.0);
    }


    public enum SlideState {
        LEVEL_ZERO,
        LOW_POSITION,
        MID_POSITION,
        HIGH_POSITION,
        IDLE
    }

    private SlideState currentState;

    public LinearSlidesUtil(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftlift");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightlift");

        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        currentState = SlideState.IDLE;
        touchSensor = hardwareMap.get(TouchSensor.class, "lifttouchsensor");

    }

    public Boolean isTouchSensorPressed() {
        return touchSensor.isPressed();
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
                if (slidesAtTargetPosition(liftLevelZero)) {
                    stopSlides();
                }
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

    public void moveToPosition(int targetPosition) {
        double ERROR_THRESHOLD = 10.0;
        leftSlide.setTargetPositionTolerance(25);
        rightSlide.setTargetPositionTolerance(25);
        leftSlide.setTargetPosition(targetPosition);
        rightSlide.setTargetPosition(targetPosition);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(.750);
        rightSlide.setPower(.750);


    }

    public void stopSlides() {
        leftSlide.setPower(0);
        rightSlide.setPower(0);
        //resetEncoder();
        //leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder(){
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public int getLeftLiftPosition() {
        return leftSlide.getCurrentPosition();
    }

    public int getRightLiftPosition() {
        return rightSlide.getCurrentPosition();
    }

    public boolean slidesAtTargetPosition(int targetPosition) {
        // Implement logic to check if the arm is sufficiently close to the target position
        // Return true when the arm is at or close to the target
        return Math.abs(leftSlide.getCurrentPosition() - targetPosition) < 10
                && Math.abs(rightSlide.getCurrentPosition() - targetPosition) < 10;
    }
}
