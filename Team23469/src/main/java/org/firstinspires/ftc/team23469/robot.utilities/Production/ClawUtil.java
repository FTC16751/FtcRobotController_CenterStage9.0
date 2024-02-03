package org.firstinspires.ftc.team23469.robot.utilities.Production;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawUtil {
    private static final double WRIST_DOWN_POSITION = 0.84;
    private static final double WRIST_SCORE_POSITION =0.6;
    private Servo rightServoClaw, leftServoClaw;
    private Servo wristServo;
    private boolean clawClosed = true; // Initial state of the claw
   // private int wristState = 0; // 0: Down,2 score// 1: Grab, 2: Carry, 3: Score
    private boolean wristState = true;
    private boolean lastLeftBumperState = false;
    private boolean lastRightBumperState = false;
    static final double INCREMENT   = 0.05;     // amount to slew servo each CYCLE_MS cycle

    public ClawUtil(HardwareMap hardwareMap) {
        rightServoClaw = hardwareMap.servo.get("rclaw");
        leftServoClaw = hardwareMap.servo.get("claw");
        wristServo = hardwareMap.servo.get("wrist");
        wristServo.setDirection(Servo.Direction.REVERSE);
        lowerWrist();
        closeClaw();

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

        //leftServoClaw.setPosition(0.15); // Adjust this value based on your servo's range
        leftServoClaw.setPosition(0.95); // Adjust this value based on your servo's range

    }
    public void closeLeftClaw() {
        //leftServoClaw.setPosition(0.40); // Adjust this value based on your servo's range
        leftServoClaw.setPosition(0.80); // Adjust this value based on your servo's rangef
    }
    public void openRightClaw() {
        //rightServoClaw.setPosition(0.95); // Adjust this value based on your servo's range
        rightServoClaw.setPosition(0.26); // Adjust this value based on your servo's range

    }
    public void closeRightClaw() {
        //rightServoClaw.setPosition(0.80); // Adjust this value based on your servo's range
        rightServoClaw.setPosition(0.40); // Adjust this value based on your servo's range
    }

    public void lowerWrist(){
        wristServo.setPosition(WRIST_DOWN_POSITION);
        wristState = true;
    }
    public void raiseWrist(){
        wristServo.setPosition(WRIST_SCORE_POSITION);
        wristState = false;
    }
    public void incrementWrist(){
        double wristposition = getWristPosition();
        wristposition += INCREMENT;
        wristServo.setPosition(wristposition);
    }
    public void setWristState(boolean wristState) {
        double position = wristState ? WRIST_DOWN_POSITION : WRIST_SCORE_POSITION;
        // Set the wrist position using the calculated position value
        wristServo.setPosition(position);
    }


    public double getClawPosition() {
        return rightServoClaw.getPosition();
    }

    public double getWristPosition() {
        return wristServo.getPosition();
    }
}
