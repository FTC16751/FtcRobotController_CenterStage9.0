package org.firstinspires.ftc.teamcode.robot_utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HangerUtil {
    private DcMotor hangerMotor;
    private Servo hangerServo;

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
        hangerMotor = hardwareMap.dcMotor.get("hanger");
        hangerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hangerServo = hardwareMap.servo.get("hangerservo");
        hangerServo.setPosition(0.0);

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
        hangerMotor.setTargetPosition(12000);
        hangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangerMotor.setPower(1.0);

        while (hangerMotor.isBusy()) {
            // Wait for the motor to reach the target position
        }
        // Automatically move hangerServo to ServoUp position
        hangerServo.setPosition(MANUAL_MOVE_MAX_POSITION);

        currentState = HangerState.UP;
    }

    private void hangHanger() {
        hangerMotor.setTargetPosition(6000);
        hangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangerMotor.setPower(1.0);

        currentState = HangerState.HANG;
    }

    private void lowerHanger() {
        hangerMotor.setTargetPosition(0);
        hangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangerMotor.setPower(1.0);

        currentState = HangerState.DOWN;

        // Reset hangerServo to ServoDown position after lowering
        hangerServo.setPosition(0.0);
    }

    public void setManualMove() {
        currentState = HangerState.MANUAL_MOVE;
    }

    public void moveHangerManually(double power) {
        if (currentState == HangerState.MANUAL_MOVE) {
            hangerMotor.setPower(power);
        }
    }

    public void moveHangerServoManually(double position) {
        // Limit manual servo movement to a maximum position of 0.65
        position = Math.min(position, MANUAL_MOVE_MAX_POSITION);
        hangerServo.setPosition(position);
    }

    public HangerState getCurrentState() {
        return currentState;
    }

    public int getHangerPosition() {
        return hangerMotor.getCurrentPosition();
    }

    public double getHangerServoPosition() {
        return hangerServo.getPosition();
    }

    public void stopMotor() {
        hangerMotor.setPower(0);
    }

    public void resetEncoder() {
        hangerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
