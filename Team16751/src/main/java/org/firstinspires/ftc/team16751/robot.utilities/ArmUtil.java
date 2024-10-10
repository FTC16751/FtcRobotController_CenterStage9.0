/* P3 2021-22 season Auto Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.team16751.robot.utilities;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
// p =0.002 i = 0.1 d = 0.0001 f = 0.1
@Config
public class ArmUtil {
    public static final double SHOULDER_GOAL_POSITION_TRANSPORT = 300;

    public static double pS = 0.0015, iS = 0.00, dS = 0.0001, fS = 0.0;
    public static double pE = 0.002, iE = 0.00, dE = 0.00015, fE = 0.0;

    public double goalShoulder;
    public double goalElbow;
    public double currElbow;
    public double currShoulder;
    DcMotor shoulderLeft, shoulderRight, elbowLeft, elbowRight;
    Servo wrist;

    //The following below reads the positions for the elbow joint (joint #2)
    int elbowTransport = 0;//POSITION 4
    /* local OpMode members. */
    HardwareMap hardwareMap = null;
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;
    private PIDFController shoulderLeftController = new PIDFController(pS, iS, dS, fS);
    private PIDFController shoulderRightController = new PIDFController(pS, iS, dS,fS);
    private PIDController elbowLeftController = new PIDController(pE, iE, dE);
    private PIDController elbowRightController = new PIDController(pE, iE, dE);
    private int currentState = 0;
    private ArmState currentArmState;
    private final double ticks_in_degreeShoulder = 3895.9 / 180.0;
    static final double WRIST_INCREMENT   = 0.05;
    double  newWristposition = (0.0);
    double wristPosition;
    int shoulderLockPosition;
    int elbowLockPosition;
    boolean usePID = true;

    /* Constructor */
    public ArmUtil(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;
        //shoulder = hardwareMap.get(DcMotor.class, "Shoulder");
        //elbow = hardwareMap.get(DcMotor.class, "Elbow");
        shoulderLeft = hardwareMap.get(DcMotor.class, "shoulderLeft");
        shoulderLeft.setDirection(DcMotor.Direction.FORWARD);
        shoulderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shoulderRight = hardwareMap.get(DcMotor.class, "shoulderRight");
        shoulderRight.setDirection(DcMotor.Direction.FORWARD);
        shoulderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        elbowLeft = hardwareMap.get(DcMotor.class, "elbowLeft");
        elbowLeft.setDirection(DcMotor.Direction.FORWARD);
        elbowLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elbowRight = hardwareMap.get(DcMotor.class, "elbowRight");
        elbowRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wrist = hardwareMap.get(Servo.class, "Wrist");
        wrist.setDirection(Servo.Direction.FORWARD);
        wristPosition = 0.10;
        wrist.setPosition(wristPosition);

        currentArmState = ArmState.INIT;
    }

    private void setupMotor(DcMotor motor, String motorName, DcMotor.Direction direction) {
        motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setupServo(Servo servo, String servoName, Servo.Direction direction, double initialPosition) {
        servo = hardwareMap.get(Servo.class, servoName);
        servo.setDirection(direction);
        servo.setPosition(initialPosition);
    }

    public void setCurrentState(ArmState state) {
        currentArmState = state;
    }
    public ArmState getCurrentState() {
        return currentArmState;
    }
    public void runStateMachine() {
        // Constants for wrist positions and arm speed
        final double WRIST_POSITION_LOW = 0.0;
        final double WRIST_POSITION_MEDIUM = 0.25;
        final double WRIST_POSITION_HIGH = 0.45;
        final double WRIST_POSITION_CUSTOM = 0.30;
        final double ARM_SPEED = 0.5;

        // Switch based on the current state
        switch (currentArmState) {
            // Initialization state
            case INIT:
                currentArmState = ArmState.INIT_SET_SERVO;
                break;

            // Set servo position during initialization
            case INIT_SET_SERVO:
                wristPosition = WRIST_POSITION_HIGH;
                wrist.setPosition(wristPosition);
                currentArmState = ArmState.INIT_RAISE_ARM;
                break;

            // Raise the arm during initialization
            case INIT_RAISE_ARM:
                raiseToPosition(1, ARM_SPEED);
                if (armAtTargetPosition()) {
                    currentArmState = ArmState.IDLE;
                }
                break;

            // Transport state
            case TRANSPORT:
                currentArmState = ArmState.TRANSPORT_RAISE_ARM;
                break;

            // Raise the arm during transport
            case TRANSPORT_RAISE_ARM:
                raiseToPosition(2, ARM_SPEED);
                if (armAtTargetPosition()) {
                    currentArmState = ArmState.TRANSPORT_SET_SERVO;
                }
                break;

            // Set servo position during transport
            case TRANSPORT_SET_SERVO:
                wristPosition = WRIST_POSITION_HIGH;
                wrist.setPosition(wristPosition);
                currentArmState = ArmState.IDLE;
                break;

            // Low score state
            case LOW_SCORE:
                currentArmState = ArmState.LOW_SCORE_RAISE_ARM;
                break;

            // Raise the arm during low scoring
            case LOW_SCORE_RAISE_ARM:
                raiseToPosition(3, ARM_SPEED);
                if (armAtTargetPosition()) {
                    currentArmState = ArmState.LOW_SCORE_SET_SERVO;
                }
                break;

            // Set servo position during low scoring
            case LOW_SCORE_SET_SERVO:
                wristPosition = WRIST_POSITION_MEDIUM;
                wrist.setPosition(wristPosition);
                currentArmState = ArmState.IDLE;
                break;

            // High score state
            case HIGH_SCORE:
                currentArmState = ArmState.HIGH_SCORE_RAISE_ARM;
                break;

            // Raise the arm during high scoring
            case HIGH_SCORE_RAISE_ARM:
                raiseToPosition(4, ARM_SPEED);
                if (armAtTargetPosition()) {
                    currentArmState = ArmState.HIGH_SCORE_SET_SERVO;
                }
                break;

            // Set servo position during high scoring
            case HIGH_SCORE_SET_SERVO:
                wristPosition = WRIST_POSITION_CUSTOM;
                wrist.setPosition(wristPosition);
                currentArmState = ArmState.IDLE;
                break;

            // Back low score state
            case BACK_LOW_SCORE_AUTO:
                currentArmState = ArmState.BACK_LOW_SCORE_RAISE_ARM_AUTO;
                break;

            // Raise the arm during back low scoring
            case BACK_LOW_SCORE_RAISE_ARM_AUTO:
                raiseToPosition(5, ARM_SPEED);
                if (armAtTargetPosition()) {
                    currentArmState = ArmState.BACK_LOW_SCORE_SET_SERVO;
                }
                break;

            case BACK_LOW_SCORE_DRIVER:
                currentArmState = ArmState.BACK_LOW_SCORE_RAISE_ARM_DRIVER;
                break;

            case BACK_LOW_SCORE_RAISE_ARM_DRIVER:
                raiseToPosition(8, ARM_SPEED);
                if(armAtTargetPosition()){
                    currentArmState = ArmState.BACK_LOW_SCORE_SET_SERVO;
                }
                break;
            // Set servo position during back low scoring
            case BACK_LOW_SCORE_SET_SERVO:
                wristPosition = WRIST_POSITION_LOW;
                wrist.setPosition(wristPosition);
                currentArmState = ArmState.IDLE;
                break;

            // Hang ready state
            case HANG_READY:
                currentArmState = ArmState.HANG_READY_RAISE_ARM;
                break;

            // Raise the arm during hang ready
            case HANG_READY_RAISE_ARM:
                raiseToPosition(6, ARM_SPEED);
                if (armAtTargetPosition()) {
                    currentArmState = ArmState.IDLE;
                }
                break;

            // Hang state
            case HANG:
                currentArmState = ArmState.HANG_RAISE_ARM;
                break;

            // Raise the arm during hanging
            case HANG_RAISE_ARM:
                raiseToPosition(7, ARM_SPEED);
                if (armAtTargetPosition()) {
                    currentArmState = ArmState.IDLE;
                }
                break;

            // Increase shoulder position state
            case INCREASE_SHOULDER_POSITION:
                currentArmState = ArmState.INCREASE_SHOULDER_POSITION;
                increaseShoulderPosition(100);
                if (armAtTargetPosition()) {
                    currentArmState = ArmState.IDLE;
                }
                break;
            // Increase shoulder position state
            case DECREASE_SHOULDER_POSITION:
                currentArmState = ArmState.INCREASE_SHOULDER_POSITION;
                decreaseShoulderPosition(100);
                if (armAtTargetPosition()) {
                    currentArmState = ArmState.IDLE;
                }
                break;
            // Idle state
            case IDLE:
                currentArmState = ArmState.IDLE;
                moveArmToPositionPID(); // Retains the last place we sent the arm
                break;
            // Default case (if needed)
            default:
                // Handle default case if needed
                break;
        }
    }

    public void raiseToPosition(int positionLevel, Double targetSpeed) {
        switch (positionLevel) {
            case 1:
                goalShoulder = 0;
                goalElbow = 0;
                moveArmToPositionPID();
                break;
            case 2:
                goalShoulder = 300;
                goalElbow = 100;
                moveArmToPositionPID();
                break;
            case 3:
                goalShoulder = 1520;
                goalElbow = 100;
                moveArmToPositionPID();
                break;
            case 4:
                goalShoulder = 1690;
                goalElbow = 300;
                moveArmToPositionPID();
                break;
            case 5:
                goalShoulder = 500;
                goalElbow = 1100;
                moveArmToPositionPID();
                break;
             //auto back score
            case 6:
                goalShoulder = 1150;
                goalElbow = 430;
                moveArmToPositionPID();
                break;
            case 7:
                goalShoulder = 100;
                goalElbow = 200;
                moveArmToHangPosition();
               /* goalShoulder = 650;
                moveArmToPositionPID();
                if (armShoulderAtTargetPosition()) {
                    goalElbow = 400;
                    goalShoulder = 200;
                    moveArmToHangPosition();
                }

                */
                break;
            case 8://driver back low score
                goalShoulder=600;
                goalElbow = 900;
                moveArmToPositionPID();
                break;
            default:
                // Handle the case for other position levels or set to zero again.
                break;
        }
    } //end raise to position
    public void raiseToPositionNoPID(int positionLevel, Double targetSpeed, boolean usePID) {
        switch (positionLevel) {
            case 1:
                goalShoulder = 0;
                goalElbow = 0;
                if (!usePID) moveArmToPositionNoPID();
                else moveArmToPositionPID();
                break;
            case 2:
                goalShoulder = 300;
                goalElbow = 100;
                if (!usePID) moveArmToPositionNoPID();
                else moveArmToPositionPID();
                break;
            case 3:
                goalShoulder = 1520;
                goalElbow = 100;
                if (!usePID) moveArmToPositionNoPID();
                else moveArmToPositionPID();
                break;
            case 4:
                goalShoulder = 1690;
                goalElbow = 300;
                if (!usePID) moveArmToPositionNoPID();
                else moveArmToPositionPID();
                break;
            case 5:
                goalShoulder = 700;
                goalElbow = 1130;
                if (!usePID) moveArmToPositionNoPID();
                else moveArmToPositionPID();
                break;
            case 6:
                goalShoulder = 1150;
                goalElbow = 430;
                if (!usePID) moveArmToPositionNoPID();
                else moveArmToPositionPID();
                break;
            case 7:
                goalShoulder = 300;
                if (!usePID) moveArmToPositionNoPID();
                else moveArmToPositionPID();

                if (armShoulderAtTargetPosition()){
                    goalElbow = 100;
                    goalShoulder = 200;
                    moveArmToHangPosition();
                }
                break;
            case 8:
                goalShoulder = 600;
                goalElbow = 900;
                if(!usePID) moveArmToPositionNoPID();
                else moveArmToPositionPID();

                break;
            default:
                // Handle the case for other position levels or set to zero again.
                break;
        }
    }//end raise to position
    public void moveArmToPositionPID() {
        // Configure PID parameters for elbow movement
        elbowLeftController.setPID(pE, iE, dE);
        elbowRightController.setPID(pE, iE, dE);

        // Move elbows to the target position
        elbowLeft.setPower(elbowLeftController.calculate(elbowLeft.getCurrentPosition(), goalElbow));
        elbowRight.setPower(elbowRightController.calculate(elbowRight.getCurrentPosition(), goalElbow));

        // Check if the elbows are at the target position before moving the shoulders
        if (armAtElbowTargetPosition()) {
            // Configure PIDF parameters for shoulder movement
            shoulderLeftController.setPIDF(pS, iS, dS, fS);
            shoulderRightController.setPIDF(pS, iS, dS, fS);

            // Move shoulders to the target position
            shoulderLeft.setPower(shoulderLeftController.calculate(shoulderLeft.getCurrentPosition(), goalShoulder));
            shoulderRight.setPower(shoulderRightController.calculate(shoulderRight.getCurrentPosition(), goalShoulder));
        } else {
            // Elbows are not at target position, do nothing
        }

        // Check if the shoulders have reached the target position
        if (armShoulderAtTargetPosition()) {
            // Perform actions when both elbows and shoulders are at target positions
        } else {
            // Shoulders are not at target position, do nothing
        }

        // Check if the entire arm has reached the target position and update the state if needed
        if (armAtTargetPosition()) {
            // Update state or perform other necessary actions
        }
    }

    public void moveArmToHangPosition() {

        shoulderLeft.setPower(-1.0);
        shoulderRight.setPower(-1.0);
        sleep(3000);

        shoulderLeft.setTargetPosition(200);
        shoulderRight.setTargetPosition(200);
        shoulderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderLeft.setPower(1.0);
        shoulderRight.setPower(1.0);

        elbowLeft.setTargetPosition((int) goalElbow);
        elbowRight.setTargetPosition((int) goalElbow);
        elbowLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowLeft.setPower(1.0);
        elbowRight.setPower(1.0);

    }

    public void moveArmToPositionNoPID() {
        elbowLeft.setTargetPosition((int) goalElbow);
        elbowRight.setTargetPosition((int) goalElbow);
        elbowLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowLeft.setPower(.50);
        elbowRight.setPower(.50);

        shoulderLeft.setTargetPosition((int) goalShoulder);
        shoulderRight.setTargetPosition((int) goalShoulder);
        shoulderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderLeft.setPower(.5);
        shoulderRight.setPower(.5);
    }
    public void moveArmToPositionNoPIDv2(double targetElbow, double targetShoulder, double power) {
        // Set target position and mode for elbows
        setMotorTargetAndMode(elbowLeft, targetElbow);
        setMotorTargetAndMode(elbowRight, targetElbow);
        // Power elbows to move to the target position
        setMotorPower(elbowLeft, power);
        setMotorPower(elbowRight, power);

        // Set target position and mode for shoulders
        setMotorTargetAndMode(shoulderLeft, targetShoulder);
        setMotorTargetAndMode(shoulderRight, targetShoulder);
        // Power shoulders to move to the target position
        setMotorPower(shoulderLeft, power);
        setMotorPower(shoulderRight, power);
    }

    // Helper method to set target position and mode for a motor
    private void setMotorTargetAndMode(DcMotor motor, double targetPosition) {
        motor.setTargetPosition((int) targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Helper method to set power for a motor
    private void setMotorPower(DcMotor motor, double power) {
        motor.setPower(power);
    }

    public boolean armAtTargetPosition() {
        // Implement logic to check if the arm is sufficiently close to the target position
        // Return true when the arm is at or close to the target
        return Math.abs(shoulderLeft.getCurrentPosition() - goalShoulder) < 15
                && Math.abs(elbowLeft.getCurrentPosition() - goalElbow) < 15;
    }
    public boolean armShoulderAtTargetPosition() {
        // Implement logic to check if the arm is sufficiently close to the target position
        // Return true when the arm is at or close to the target
        return Math.abs(shoulderLeft.getCurrentPosition() - goalShoulder) < 15;
    }
    public boolean armAtElbowTargetPosition() {
        // Implement logic to check if the arm is sufficiently close to the target position
        // Return true when the arm is at or close to the target
        return(Math.abs(elbowLeft.getCurrentPosition() - goalElbow) < 15);
    }
    public void increaseElbowPosition(int increaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = elbowLeft.getCurrentPosition();
        newPosition = currentPosition + increaseAmount;
        changeElbowPosition(newPosition);
    }

    public void decreaseElbowPosition(int decreaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = elbowLeft.getCurrentPosition();
        newPosition = currentPosition - decreaseAmount;
        changeElbowPosition(newPosition);
    }

    public void changeElbowPosition(int newPosition) {
        elbowLeft.setTargetPosition(newPosition);
        elbowLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowLeft.setPower(1.0);
        elbowRight.setTargetPosition(newPosition);
        elbowRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowRight.setPower(1.0);
    }

    public void increaseShoulderPosition(int increaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = shoulderLeft.getCurrentPosition();
        newPosition = currentPosition + increaseAmount;
        changeShoulderPosition(newPosition);
    }
    public void decreaseShoulderPosition(int decreaseAmount) {
        decreaseAmount = 100;
        int lcurrentPosition;
        int rcurrentPosition;
        int newlPosition;
        int newrPosition;
        lcurrentPosition = shoulderLeft.getCurrentPosition();
        rcurrentPosition = shoulderRight.getCurrentPosition();
        newlPosition = lcurrentPosition - decreaseAmount;
        newrPosition = rcurrentPosition - decreaseAmount;
        changeShoulderPosition(newlPosition,newrPosition);

    }
    public void changeShoulderPosition(int newlPosition, int newrPosition) {
        shoulderLeft.setTargetPosition(newlPosition);
        shoulderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderLeft.setPower(0.25);

        shoulderRight.setTargetPosition(newrPosition);
        shoulderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderRight.setPower(0.25);

    }
    public void changeShoulderPosition(int newPosition) {
        shoulderLeft.setTargetPosition(newPosition);
        shoulderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderLeft.setPower(1.0);

        shoulderRight.setTargetPosition(newPosition);
        shoulderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderRight.setPower(1.0);

    }

    public void setElbowMotorPower(double v) {
        elbowLeft.setPower(v);
    }

    public void setShoulderMotorPower(double v) {
        shoulderLeft.setPower(v);
    }

    public void stopArm() {
        setMotorPower(0);
    }//StopArm

    public double getMotorPower() {
        return elbowLeft.getPower();
    }

    public void setMotorPower(double v) {
        elbowLeft.setPower(v);
    }

    public int getMotorPosition() {
        return elbowLeft.getCurrentPosition();
    }

    public int getLeftElbowMotorPosition() {
        return elbowLeft.getCurrentPosition();
    }
    public int getRightElbowMotorPosition() {
        return elbowRight.getCurrentPosition();
    }


    public int getLeftShoulderMotorPosition() {
        return shoulderLeft.getCurrentPosition();
    }
    public int getRightShoulderMotorPosition() {
        return shoulderRight.getCurrentPosition();
    }

    public void setWristPosition(double position) {
        wrist.setPosition(position);

    }
    public void incrementWristPosition () {
        wristPosition += 0.005;
        wristPosition = Math.min(1.0,wristPosition);
        double currWristPos = wrist.getPosition();
        newWristposition = currWristPos + wristPosition;
        wrist.setPosition(newWristposition);
    }
    public void decrementWristPosition () {
        wristPosition += 0.009;
        wristPosition = Math.max(0.0,wristPosition);
        //wrist.setPosition(wristPosition);
        double currWristPos = wrist.getPosition();
        newWristposition = currWristPos - wristPosition;
        wrist.setPosition(newWristposition);
    }

    public double getWristPosition(){ return wrist.getPosition();}

    public void setMotorMode(int mode) {
        if (mode == 1) elbowLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (mode == 2) elbowLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetArmEncoders() {
        resetElbowEncoders();
        resetShoulderEncoders();
    }

    public void resetElbowEncoders(){
        elbowLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetShoulderEncoders(){
        shoulderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setArmMotorRunMode() {
        shoulderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopAndResetArmMotors() {
        resetArmEncoders();
        setArmMotorRunMode();
    }
    public void stopShoulderMotors() {
        shoulderLeft.setPower(0);
        shoulderRight.setPower(0);
    }
    public void stopElbowMotors() {
        elbowLeft.setPower(0);
        elbowRight.setPower(0);
    }
    public enum ArmState {
        INIT,
        INIT_SET_SERVO,
        INIT_RAISE_ARM,
        TRANSPORT,
        TRANSPORT_RAISE_ARM,
        TRANSPORT_SET_SERVO,
        LOW_SCORE,
        LOW_SCORE_RAISE_ARM,
        LOW_SCORE_SET_SERVO,
        HIGH_SCORE,
        HIGH_SCORE_RAISE_ARM,
        HIGH_SCORE_SET_SERVO,
        BACK_LOW_SCORE_AUTO,
        BACK_LOW_SCORE_RAISE_ARM_AUTO,
        BACK_LOW_SCORE_DRIVER,
        BACK_LOW_SCORE_RAISE_ARM_DRIVER,
        BACK_LOW_SCORE_SET_SERVO,
        IDLE,
        HANG_READY_RAISE_ARM,
        HANG_READY,
        HANG_RAISE_ARM,
        INCREASE_SHOULDER_POSITION,
        DECREASE_SHOULDER_POSITION,
        HANG
    }
    private final int[] armMoveOrder = {1, 2, 3};
}   //end program
