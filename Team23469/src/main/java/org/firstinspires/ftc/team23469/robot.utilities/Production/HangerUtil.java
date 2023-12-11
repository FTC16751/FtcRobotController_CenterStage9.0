package org.firstinspires.ftc.team23469.robot.utilities.Production;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HangerUtil {
    private DcMotor hanger;
    private Servo rhangerServo;
    private Servo lhangerServo;

    enum HangerState {
        DOWN,
        UP,
        HANG,
        MANUAL_MOVE
    }

    private HangerState currentState;
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

        currentState = HangerState.DOWN;
    }

    public void toggleHanger() {
        switch (currentState) {
            case DOWN:
                raiseHanger();
                break;
            case UP:
                hangHanger();
                break;
            case HANG:
                lowerHanger();
                break;
            case MANUAL_MOVE:
                break;
            default:
                break;
        }
    }

    private void raiseHanger() {
        hanger.setTargetPosition(12000);
        hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hanger.setPower(1.0);

        while (hanger.isBusy()) {
            // Wait for the motor to reach the target position
        }
        // Automatically move hangerServo to ServoUp position
        rhangerServo.setPosition(.45);
        lhangerServo.setPosition(.80);

        currentState = HangerState.UP;
    }

    private void hangHanger() {
        currentState = HangerState.HANG;
        hanger.setTargetPosition(2000);
        hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hanger.setPower(1.0);
        while (hanger.isBusy()) {
            // Wait for the motor to reach the target position
        }

    }

    private void lowerHanger() {
        currentState = HangerState.DOWN;
        hanger.setTargetPosition(0);
        hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hanger.setPower(1.0);
        while (hanger.isBusy()) {
            // Wait for the motor to reach the target position
        }
        rhangerServo.setPosition(0.8);
        lhangerServo.setPosition(0.5);

    }

    public void setManualMove() {
        currentState = HangerState.MANUAL_MOVE;
    }

    public void moveHangerManually(double power) {
        if (currentState == HangerState.MANUAL_MOVE) {
            hanger.setPower(power);
        }
    }

    public void moveHangerServoManually(double position) {
        // Limit manual servo movement to a maximum position of 0.65
        position = Math.min(position, MANUAL_MOVE_MAX_POSITION);
        rhangerServo.setPosition(position);
    }

    public HangerState getCurrentState() {
        return currentState;
    }

    public int getHangerPosition() {
        return hanger.getCurrentPosition();
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
}
