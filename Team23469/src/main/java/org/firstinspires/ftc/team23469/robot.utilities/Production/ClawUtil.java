package org.firstinspires.ftc.team23469.robot.utilities.Production;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawUtil {
    private Servo servoClaw;
    private Servo wristServo;
    private boolean clawClosed = true; // Initial state of the claw
   // private int wristState = 0; // 0: Down,2 score// 1: Grab, 2: Carry, 3: Score
    private boolean wristState = true;
    private boolean lastLeftBumperState = false;
    private boolean lastRightBumperState = false;
    public ClawUtil(HardwareMap hardwareMap) {
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
        servoClaw.setPosition(0.50); // Adjust this value based on your servo's range
        clawClosed = false;
    }

    public void closeClaw() {
        servoClaw.setPosition(0.85); // Adjust this value based on your servo's range
        clawClosed = true;
    }
    public void lowerWrist(){
        wristServo.setPosition(0.01);
        wristState = true;
    }
    public void raiseWrist(){
        wristServo.setPosition(0.2);
        wristState = false;
    }
    public void setWristState(boolean wristState) {
        double position = wristState ? 0.01 : 0.2;
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
        return servoClaw.getPosition();
    }

    public double getWristPosition() {
        return wristServo.getPosition();
    }
}
