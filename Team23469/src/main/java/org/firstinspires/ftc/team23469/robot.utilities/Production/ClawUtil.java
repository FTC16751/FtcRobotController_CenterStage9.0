package org.firstinspires.ftc.team23469.robot.utilities.Production;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawUtil {
    private Servo rightServoClaw, leftServoClaw;
    private Servo wristServo;
    private boolean clawClosed = true; // Initial state of the claw
   // private int wristState = 0; // 0: Down,2 score// 1: Grab, 2: Carry, 3: Score
    private boolean wristState = true;
    private boolean lastLeftBumperState = false;
    private boolean lastRightBumperState = false;
    public ClawUtil(HardwareMap hardwareMap) {
        rightServoClaw = hardwareMap.servo.get("claw");
        leftServoClaw = hardwareMap.servo.get("rclaw");
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
            // Toggle the wrist state when right bumper is pressed
            wristState = !wristState;
            setWristState(wristState);
        }

        lastRightBumperState = rightBumper;
    }
    /*public void toggleWristWithBumper(boolean rightBumper) {
    boolean rightBumperEdge = rightBumper && !lastRightBumperState;

    if (rightBumperEdge) {
        // Increment wrist state when right bumper is pressed
        wristState = (wristState + 1) % 4;
        setWristState(wristState);
    }

    lastRightBumperState = rightBumper;
}

 */
    public void openClaw() {
        openLeftClaw();
        openRightClaw();
        clawClosed = false;
    }

    public void closeClaw() {
        closeLeftClaw();
        closeRightClaw();
        clawClosed = true;
    }

    public void openLeftClaw() {
        leftServoClaw.setPosition(0.30); // Adjust this value based on your servo's range
    }
    public void closeLeftClaw() {
        leftServoClaw.setPosition(0.15); // Adjust this value based on your servo's range
    }
    public void openRightClaw() {
        rightServoClaw.setPosition(0.70); // Adjust this value based on your servo's range
    }
    public void closeRightClaw() {
        rightServoClaw.setPosition(0.95); // Adjust this value based on your servo's range
    }

    public void lowerWrist(){
        wristServo.setPosition(0.65);
        wristState = true;
    }
    public void raiseWrist(){
        wristServo.setPosition(0.4);
        wristState = false;
    }
    public void setWristState(boolean wristState) {
        double position = wristState ? 0.65 : 0.4;// : 0.2;
        // Set the wrist position using the calculated position value
        wristServo.setPosition(position);
    }
    /*
    public void setWristState(int state) {
        switch (state) {
            case 0: // Down
                wristServo.setPosition(0.01); // Adjust this value based on your servo's range
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
    */

    public double getClawPosition() {
        return rightServoClaw.getPosition();
    }

    public double getWristPosition() {
        return wristServo.getPosition();
    }
}
