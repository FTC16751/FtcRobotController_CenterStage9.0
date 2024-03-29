package org.firstinspires.ftc.team23469.robot.utilities.Production;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HangerUtil {
    private DcMotor hanger;
    private Servo rhangerServo;
    private Servo lhangerServo;

    public enum HangerState {
        RASIE_HANGER,
        HANG_HANGER,
        LOWER_HANGER,
        RAISE_HANGER_SERVO,
        LOWER_HANGER_SERVO,
        MANUAL_MOVE,
        IDLE
    }

    public enum HangerServoState {
        RAISE_HANGER_SERVO,
        LOWER_HANGER_SERVO
    }

    private HangerState currentState;
    public HangerServoState currentServoState = HangerServoState.LOWER_HANGER_SERVO;
    private static final double MANUAL_MOVE_MAX_POSITION = 0.65;
    private static final double MANUAL_MOVE_INCREMENT = 0.05;

    public HangerUtil(HardwareMap hardwareMap) {
        hanger = hardwareMap.dcMotor.get("hanger");
        hanger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rhangerServo = hardwareMap.servo.get("righthangerservo");
        rhangerServo.setPosition(0.8);
        lhangerServo = hardwareMap.servo.get("lefthangerservo");
        lhangerServo.setPosition(0.5);

        currentState = HangerState.RASIE_HANGER;
    }

    public void toggleHanger() {
        switch (currentState) {
            case RASIE_HANGER:
                raiseHanger();
                break;
            case HANG_HANGER:
                hangHanger();
                break;
            case LOWER_HANGER:
                lowerHanger();
                break;
            case RAISE_HANGER_SERVO:
                raiseHangerServo();
                break;
            case LOWER_HANGER_SERVO:
                lowerHangerServo();
                break;
            case MANUAL_MOVE:
                break;
            case IDLE:
                break;
            default:
                break;
        }
    }

    public void raiseHanger() {
        currentState = HangerState.RASIE_HANGER;
        hanger.setTargetPosition(12000);
        hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hanger.setPower(1.0);
    }

    public void hangHanger() {
        currentState = HangerState.HANG_HANGER;
        hanger.setTargetPosition(2000);
        hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hanger.setPower(1.0);
    }

    public void lowerHanger() {
        currentState = HangerState.LOWER_HANGER;
        hanger.setTargetPosition(0);
        hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hanger.setPower(1.0);
    }

    public void raiseHangerServo() {
        currentServoState = HangerServoState.RAISE_HANGER_SERVO;
        //move hangerServo to ServoUp position
        rhangerServo.setPosition(.45);
        lhangerServo.setPosition(.80);
    }

    public void lowerHangerServo() {
        currentServoState = HangerServoState.LOWER_HANGER_SERVO;
        rhangerServo.setPosition(0.8);
        lhangerServo.setPosition(0.5);
    }

    public void setManualMove() {
        currentState = HangerState.MANUAL_MOVE;
    }

    public void moveHangerManually(double power) {
       currentState = HangerState.MANUAL_MOVE;
       hanger.setPower(power);
    }

    public void moveHangerServoManually(double position) {
        // Limit manual servo movement to a maximum position of 0.65
        position = Math.min(position, MANUAL_MOVE_MAX_POSITION);
        rhangerServo.setPosition(position);
    }

    public HangerState getCurrentState() {
        return currentState;
    }

    public void setCurrentState(HangerState newState) {
        currentState = newState;
    }

    public int getHangerPosition() {
        return hanger.getCurrentPosition();
    }

    public double getHangerPower() {
        return hanger.getPower();
    }

    public double getHangerServoPosition() {
        return rhangerServo.getPosition();
    }

    public void stopMotor() {
        hanger.setPower(0);
    }

    public void resetEncoder() {
        hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public boolean hangerAtTargetPosition(int targetPosition) {
        // Implement logic to check if sufficiently close to the target position
        // Return true when at or close to the target
        return Math.abs(hanger.getCurrentPosition() - targetPosition) < 15;
    }
}
