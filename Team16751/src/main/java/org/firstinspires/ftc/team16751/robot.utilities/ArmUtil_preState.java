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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class ArmUtil_preState {
    TouchSensor touch;
    TouchSensor touch2;
    DistanceSensor distance;
    public static final double SHOULDER_GOAL_POSITION_TRANSPORT = 300;

    public static double pS = 0.0015, iS = 0.00, dS = 0.0001, fS = 0.0;
    public static double pE = 0.002, iE = 0.00, dE = 0.00015, fE = 0.0;

    public double goalShoulder;
    public double goalElbow;
    public double currElbow;
    public double currShoulder;
    public DcMotorEx shoulderLeft, shoulderRight, elbowLeft, elbowRight;
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
    private ArmState priorArmState;

    private final double ticks_in_degreeShoulder = 3895.9 / 180.0;
    static final double WRIST_INCREMENT   = 0.05;
    double  newWristposition = (0.0);
    double wristPosition;
    int shoulderLockPosition;
    int elbowLockPosition;
    boolean usePID = true;

    /* Constructor */
    public ArmUtil_preState(LinearOpMode opmode) {
        myOpMode = opmode;
    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;
        //shoulder = hardwareMap.get(DcMotorEx.class, "Shoulder");
        //elbow = hardwareMap.get(DcMotorEx.class, "Elbow");
        shoulderLeft = hardwareMap.get(DcMotorEx.class, "shoulderLeft");
        shoulderLeft.setDirection(DcMotorEx.Direction.REVERSE);
        shoulderLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shoulderLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shoulderLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shoulderRight = hardwareMap.get(DcMotorEx.class, "shoulderRight");
        shoulderRight.setDirection(DcMotorEx.Direction.FORWARD);
        //shoulderRight.setDirection(DcMotorEx.Direction.REVERSE);
        shoulderRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shoulderRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shoulderRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        elbowLeft = hardwareMap.get(DcMotorEx.class, "elbowLeft");
        elbowLeft.setDirection(DcMotorEx.Direction.REVERSE);
        elbowLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elbowLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elbowLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        elbowRight = hardwareMap.get(DcMotorEx.class, "elbowRight");
        elbowRight.setDirection(DcMotorEx.Direction.FORWARD);
        elbowRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elbowRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elbowRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        wrist = hardwareMap.get(Servo.class, "Wrist");
        wrist.setDirection(Servo.Direction.FORWARD);
        wristPosition = 0.10;
        wrist.setPosition(wristPosition);

        currentArmState = ArmState.INIT;


        // Get the touch sensor and motor from hardwareMap
    //    touch = hardwareMap.get(TouchSensor.class, "lmagswitch");
      //  touch2 = hardwareMap.get(TouchSensor.class, "rmagswitch");
    }

 /*   public void autoHomeArm() {

        while (!touch.isPressed()) {
            setMotorPower(shoulderLeft,.25);
            setMotorPower(shoulderRight,.25);
        }
        //stopShoulderMotors();
        stopAndResetArmMotors();
    }*/

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
                wristPosition = WRIST_POSITION_HIGH;
                wrist.setPosition(wristPosition);

                raiseToPosition(1, ARM_SPEED);
                if (armAtTargetPosition()) {
                    currentArmState = ArmState.IDLE;
                }
                priorArmState = ArmState.INIT;
                break;

            // Transport state
            case TRANSPORT:
                wristPosition = WRIST_POSITION_HIGH;
                wrist.setPosition(wristPosition);
                /*
                if (priorArmState == ArmState.BACK_LOW_SCORE_DRIVER) {
                    wristPosition = WRIST_POSITION_HIGH;
                }
                else if (priorArmState == ArmState.BACK_LOW_SCORE_AUTO) {
                    wristPosition = WRIST_POSITION_HIGH;
                } else {
                    wristPosition = .13;
                }

                wrist.setPosition(wristPosition);
*/
                raiseToPosition(2, ARM_SPEED);
                if (armAtTargetPosition()) {
                    wristPosition = .13;
                    wrist.setPosition(wristPosition);

                    if (priorArmState == ArmState.BACK_LOW_SCORE_DRIVER) {
                        wristPosition = .13;
                        wrist.setPosition(wristPosition);
                    }
                    currentArmState = ArmState.IDLE;
                }
                priorArmState = ArmState.TRANSPORT;
                break;

            case SECOND_STACK:
                raiseToPosition(10,ARM_SPEED);
                if(armAtTargetPosition()){
                    wristPosition = WRIST_POSITION_HIGH;
                    wrist.setPosition(wristPosition);
                    currentArmState = ArmState.IDLE;
                }
                break;

            case THIRD_STACK:
                raiseToPosition(11,ARM_SPEED);
                if(armAtTargetPosition()){
                    wristPosition = WRIST_POSITION_HIGH;
                    wrist.setPosition(wristPosition);
                    currentArmState = ArmState.IDLE;
                }
                break;

            case FOURTH_STACK:
                raiseToPosition(12,ARM_SPEED);
                if(armAtTargetPosition()){
                    wristPosition = WRIST_POSITION_HIGH;
                    wrist.setPosition(wristPosition);
                    currentArmState = ArmState.IDLE;
                }
                break;

            case TOP_STACK:
                raiseToPosition(13,ARM_SPEED);
                if(armAtTargetPosition()){
                    wristPosition = WRIST_POSITION_HIGH;
                    wrist.setPosition(wristPosition);
                    currentArmState = ArmState.IDLE;
                }
                break;

            case LOW_SCORE:
                raiseToPosition(3, ARM_SPEED);
                if (armAtTargetPosition()) {
                    wristPosition = WRIST_POSITION_MEDIUM;
                    wrist.setPosition(wristPosition);
                    currentArmState = ArmState.IDLE;
                }
                priorArmState = ArmState.LOW_SCORE;
                break;

            // High score state
            case HIGH_SCORE:
                raiseToPosition(4, ARM_SPEED);
                if (armAtTargetPosition()) {
                    wristPosition = WRIST_POSITION_CUSTOM;
                    wrist.setPosition(wristPosition);
                    currentArmState = ArmState.IDLE;
                }
                priorArmState = ArmState.HIGH_SCORE;
                break;

            // Back low score state
            case BACK_LOW_SCORE_AUTO:
                raiseToPosition(5, ARM_SPEED);
                if (armAtTargetPosition()) {
                    wristPosition = WRIST_POSITION_LOW;
                    wrist.setPosition(wristPosition);
                    currentArmState = ArmState.IDLE;
                }
                priorArmState = ArmState.BACK_LOW_SCORE_AUTO;
                break;

            // Raise the arm during back low scorin
            case BACK_LOW_SCORE_DRIVER:
                raiseToPosition(8, ARM_SPEED);
                if(armAtTargetPosition()){
                    wristPosition = WRIST_POSITION_LOW;
                    wrist.setPosition(wristPosition);
                    currentArmState = ArmState.IDLE;
                }
                priorArmState = ArmState.BACK_LOW_SCORE_DRIVER;
                break;

            // Hang ready state
            case HANG_READY:
                raiseToPosition(6, ARM_SPEED);
                if (armAtTargetPosition()) {
                    currentArmState = ArmState.IDLE;
                }
                priorArmState = ArmState.HANG_READY;
                break;

            // Hang state
            case HANG:
                raiseToPosition(7, ARM_SPEED);
                if (armAtTargetPosition()) {
                    currentArmState = ArmState.IDLE;
                }
                priorArmState = ArmState.HANG;
                break;

            // Increase shoulder position state
            case INCREASE_SHOULDER_POSITION:
                currentArmState = ArmState.INCREASE_SHOULDER_POSITION;
                increaseShoulderPosition(50);
                if (armAtTargetPosition()) {
                    currentArmState = ArmState.IDLE;
                }
                break;

            // decrease shoulder position state
            case DECREASE_SHOULDER_POSITION:
                currentArmState = ArmState.INCREASE_SHOULDER_POSITION;
                decreaseShoulderPosition(50);
                if (armAtTargetPosition()) {
                    currentArmState = ArmState.IDLE;
                }
                break;

            // Idle state--keeps arm at current position
            case IDLE:
                currentArmState = ArmState.IDLE;
                //moveArmToPositionPID(); // Retains the last place we sent the arm
                break;

            case AUTOSTART:
                raiseToPosition(9,ARM_SPEED);
                if(armAtTargetPosition()){
                    currentArmState=ArmState.IDLE;
                }
                break;

            case LOWER_SCORE:
                wrist.setPosition(WRIST_POSITION_MEDIUM);
                raiseToPosition(14,ARM_SPEED);
                if(armAtTargetPosition()){
                    wrist.setPosition(0.9);
                    currentArmState = ArmState.IDLE;
                }
                priorArmState = ArmState.LOWER_SCORE;
                break;

            default:
                // Handle default case if needed
                break;
        }
    }

    public void raiseToPosition(int positionLevel, Double targetSpeed) {
        switch (positionLevel) {
            case 1: //init
                goalShoulder = 0;
                goalElbow = 0;
                moveArmToPositionNoPIDwPower(goalElbow,goalShoulder,.25,.50);
                break;
            case 2: //transport
                goalShoulder = 200;
                goalElbow = 115;
                moveArmToPositionNoPIDwPower(goalElbow,goalShoulder,.40,.75);
                break;
            case 3: //'mid' score
                //goalShoulder = 1520;
                goalShoulder =1818;
                goalElbow = 100;
                moveArmToPositionNoPIDwPower(goalElbow,goalShoulder,1,1);
                break;
            case 4: //high score
               //goalShoulder = 1690;
                goalShoulder = 1992;
                goalElbow = 300;
                if (priorArmState == ArmState.BACK_LOW_SCORE_DRIVER) {
                    moveArmToPositionNoPIDwPower(goalElbow,goalShoulder,.5,.5);
                } else
                moveArmToPositionNoPIDwPower(goalElbow,goalShoulder,1,1);
                break;
            case 5://auto back score
                goalShoulder = 600;
                //goalShoulder = 900;
                goalElbow = 950;
                moveArmToPositionNoPIDwPower(goalElbow,goalShoulder,.5,.75);
                break;

            case 6: //hang ready
                //goalShoulder = 1150;
                goalShoulder = 1650;
                goalElbow = 430;
                moveArmToPositionNoPIDwPower(goalElbow,goalShoulder,1,1);
                break;
            case 7: //pull up hang
                goalShoulder = 800;
                goalElbow = 200;
                moveArmToPositionNoPIDwPower(goalElbow,goalShoulder,.5,.50);
                break;
            case 8://driver back low score
               // goalShoulder=600;
                goalShoulder=1218;

                goalElbow = 1000;
                moveArmToPositionNoPIDwPower(goalElbow,goalShoulder,.50,1);
                break;
            case 9://auto start position
                goalShoulder=200;
                goalShoulder=500;
                goalElbow = 50;
                moveArmToPositionNoPIDwPower(goalElbow,goalShoulder,.50,.50);
                break;
            case 10://second pixel
                goalShoulder=25;
                goalElbow=20;
                moveArmToPositionNoPIDwPower(goalElbow,goalShoulder,1,1);
                break;
            case 11://third pixel
                goalShoulder=70;
                goalElbow=20;
                moveArmToPositionNoPIDwPower(goalElbow,goalShoulder,1,1);
                break;
            case 12://fourth pixel
                goalShoulder=120;
                goalElbow=20;
                moveArmToPositionNoPIDwPower(goalElbow,goalShoulder,1,1);
                break;
            case 13://top pixel
                goalShoulder=170;
                goalElbow=21;
                moveArmToPositionNoPIDwPower(goalElbow,goalShoulder,1.0,1.0);
                break;
            case 14:
                goalShoulder = 1898;
                goalElbow = -62;
                moveArmToPositionNoPIDwPower(goalElbow,goalShoulder,1,1);

                break;
            default:
                // Handle the case for other position levels or set to zero again.
                break;
        }
    } //end raise to position

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

    }

    public void moveArmToPositionNoPIDwPower(double targetElbow, double targetShoulder, double elbowPower, double shoulderPower) {
        // Set target position and mode for elbows
        setMotorTargetAndMode(elbowLeft, targetElbow);
        setMotorTargetAndMode(elbowRight, targetElbow);
        // Power elbows to move to the target position
        setMotorPower(elbowLeft, elbowPower);
        setMotorPower(elbowRight, elbowPower);

        // Set target position and mode for shoulders
        setMotorTargetAndMode(shoulderLeft, targetShoulder);
        setMotorTargetAndMode(shoulderRight, targetShoulder);
        // Power shoulders to move to the target position
        setMotorPower(shoulderLeft, shoulderPower);
        setMotorPower(shoulderRight, shoulderPower);
    }

    // Helper method to set target position and mode for a motor
    private void setMotorTargetAndMode(DcMotorEx motor, double targetPosition) {
        motor.setTargetPosition((int) targetPosition);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    // Helper method to set power for a motor
    public void setMotorPower(DcMotor motor, double power) {
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
        elbowLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elbowLeft.setPower(1.0);
        elbowRight.setTargetPosition(newPosition);
        elbowRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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
        //decreaseAmount = 50;
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
        shoulderLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        shoulderLeft.setPower(1.0);

        shoulderRight.setTargetPosition(newrPosition);
        shoulderRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        shoulderRight.setPower(1.0);

    }
    public void changeShoulderPosition(int newPosition) {
        shoulderLeft.setTargetPosition(newPosition);
        shoulderLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        shoulderLeft.setPower(1.0);

        shoulderRight.setTargetPosition(newPosition);
        shoulderRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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
    public double getLeftShoulderCurrent() {
        return shoulderLeft.getCurrent(CurrentUnit.AMPS);
    }
    public double getRightShoulderCurrent() {
        return shoulderRight.getCurrent(CurrentUnit.AMPS);
    }
    public double getLeftElbowCurrent() {
        return elbowLeft.getCurrent(CurrentUnit.AMPS);
    }
    public double getRightElbowCurrent() {
        return elbowRight.getCurrent(CurrentUnit.AMPS);
    }
    public void setWristPosition(double position) {
        wrist.setPosition(position);

    }
    public void incrementWristPosition () {
        //wristPosition = Math.min(1.0,WRIST_INCREMENT);
        double currWristPos = wrist.getPosition();
        newWristposition = currWristPos + WRIST_INCREMENT;
        wrist.setPosition(newWristposition);
    }
    public void decrementWristPosition () {
        wristPosition = Math.max(0.0,WRIST_INCREMENT);
        //wrist.setPosition(wristPosition);
        double currWristPos = wrist.getPosition();
        newWristposition = currWristPos - WRIST_INCREMENT;
        wrist.setPosition(newWristposition);
    }

    public double getWristPosition(){ return wrist.getPosition();}

    public void setMotorMode(int mode) {
        if (mode == 1) elbowLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        if (mode == 2) elbowLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetArmEncoders() {
        resetElbowEncoders();
        resetShoulderEncoders();
    }

    public void resetElbowEncoders(){
        elbowLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elbowRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetShoulderEncoders(){
        shoulderLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shoulderRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setArmMotorRunMode() {
        shoulderLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shoulderRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elbowLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elbowRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
        HANG,
        AUTOSTART,
        SECOND_STACK,
        THIRD_STACK,
        FOURTH_STACK,
        TOP_STACK,
        LOWER_SCORE
    }

}   //end program
