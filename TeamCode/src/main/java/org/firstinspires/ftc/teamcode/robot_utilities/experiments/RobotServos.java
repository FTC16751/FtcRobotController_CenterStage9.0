package org.firstinspires.ftc.teamcode.robot_utilities.experiments;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotServos {
    private Servo servoClaw;
    private Servo wristServo;
    private boolean clawClosed = true; // Initial state of the claw
    private int wristState = 0; // 0: Down, 1: Grab, 2: Carry, 3: Score
    private boolean lastLeftBumperState = false;
    private boolean lastRightBumperState = false;
    public RobotServos(HardwareMap hardwareMap) {
        servoClaw = hardwareMap.servo.get("claw");
        wristServo = hardwareMap.servo.get("wrist");
        wristServo.setDirection(Servo.Direction.REVERSE);

    }

    public void toggleClawWithBumper(boolean leftBumper) {
        boolean leftBumperEdge = leftBumper && !lastLeftBumperState;

        if (leftBumperEdge) {
            // Toggle the claw state when the left bumper is pressed
            if (clawClosed) {
                openClaw();
            } else {
                closeClaw();
            }
        }

        lastLeftBumperState = leftBumper;
    }
    public void toggleWristWithBumper(boolean rightBumper) {
        boolean rightBumperEdge = rightBumper && !lastRightBumperState;

        if (rightBumperEdge) {
            // Increment wrist state when right bumper is pressed
            wristState = (wristState + 1) % 4;
            setWristState(wristState);
        }

        lastRightBumperState = rightBumper;
    }
    private void openClaw() {
        servoClaw.setPosition(0.50); // Adjust this value based on your servo's range
        clawClosed = false;
    }

    private void closeClaw() {
        servoClaw.setPosition(0.75); // Adjust this value based on your servo's range
        clawClosed = true;
    }

    public void setWristState(int state) {
        switch (state) {
            case 0: // Down
                wristServo.setPosition(0.0); // Adjust this value based on your servo's range
                break;
            case 1: // Grab
                wristServo.setPosition(0.1); // Adjust this value based on your servo's range
                break;
            case 2: // Carry
                wristServo.setPosition(0.3); // Adjust this value based on your servo's range
                break;
            case 3: // Score
                wristServo.setPosition(0.2); // Adjust this value based on your servo's range
                break;
            default:
                break;
        }
    }

    public double getClawPosition() {
        return servoClaw.getPosition();
    }

    public double getWristPosition() {
        return wristServo.getPosition();
    }
}
