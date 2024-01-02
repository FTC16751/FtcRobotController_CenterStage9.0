/* P3 2021-22 season Auto Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.team16751.robot.utilities;

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
    int shoulderLockPosition;
    int elbowLockPosition;

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
        elbowRight.setDirection(DcMotor.Direction.REVERSE);
        elbowRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wrist = hardwareMap.get(Servo.class, "Wrist");
        wrist.setDirection(Servo.Direction.FORWARD);

        wrist.setPosition(.35);

        currentArmState = ArmState.INIT;
    }

    public ArmState getCurrentState() {
        return currentArmState;
    }

    public void setCurrentState(ArmState state) {
        currentArmState = state;
    }

    public void runStateMachine() {
        switch (currentArmState) {
            case INIT:
                currentArmState = ArmState.INIT_SET_SERVO;
                break;
            case INIT_SET_SERVO:
                wrist.setPosition(0.35);
                currentArmState = ArmState.INIT_RAISE_ARM;
                break;
            case INIT_RAISE_ARM:
                currentArmState = ArmState.INIT_RAISE_ARM;
                raiseToPosition(1, .5);
                if (armAtTargetPosition()) {
                    currentArmState = ArmState.IDLE;
                }
                break;
            case TRANSPORT:
                currentArmState = ArmState.TRANSPORT_RAISE_ARM;
                break;
            case TRANSPORT_RAISE_ARM:
                currentArmState = ArmState.TRANSPORT_RAISE_ARM;
                raiseToPosition(2, .5);
                if (armAtTargetPosition()) {
                    shoulderLockPosition = shoulderLeft.getCurrentPosition();
                    elbowLockPosition = elbowLeft.getCurrentPosition();
                    currentArmState = ArmState.TRANSPORT_SET_SERVO;
                }
                break;
            case TRANSPORT_SET_SERVO:
                currentArmState = ArmState.TRANSPORT_SET_SERVO;
                wrist.setPosition(.45);
                currentArmState = ArmState.IDLE;
                break;
            case LOW_SCORE:
                currentArmState = ArmState.LOW_SCORE;
                raiseToPosition(3, .5);
                break;
            case HIGH_SCORE:
                currentArmState = ArmState.HIGH_SCORE;
                raiseToPosition(4, .5);
                break;
            case BACK_LOW_SCORE:
                currentArmState = ArmState.BACK_LOW_SCORE;
                raiseToPosition(5, .5);
                break;
            case IDLE:
                currentArmState = ArmState.IDLE;
                moveArmToPosition(); //should retain the last place we sent the arm
                break;
            default:
                break;
        }
    }

    public void raiseToPosition(int positionLevel, Double targetSpeed) {
        if (positionLevel == 1) //rest
        {
            goalShoulder = 0;
            goalElbow = 0;
            moveArmToPosition();

        } else if (positionLevel == 2) //transport
        {
            wrist.setPosition(0.45);
            goalShoulder = 300;
            goalElbow = 200;
            moveArmToPosition();
            if (armAtTargetPosition()) {
                wrist.setPosition(0.0);
            }
        } else if (positionLevel == 3) //score low
        {
            goalShoulder = 1520;
            goalElbow =100;
            moveArmToPosition();
            if (armAtTargetPosition()){
                wrist.setPosition(0.15);
            }
        } else if (positionLevel == 4) //score high
        {
            goalShoulder = 1690;
            goalElbow = 300;
            moveArmToPosition();
            if (armAtTargetPosition()){
                wrist.setPosition(0.2);
            }
        } else if (positionLevel == 5) //Back low score
        {
            wrist.setPosition(0.45);
            goalShoulder = 400;
            goalElbow = 900;
            moveArmToPosition();
            if (armShoulderAtTargetPosition()){
                wrist.setPosition(0.0);
            }
        }else {
            //zero again!
        }
    } //end raise to position

    public void moveArmToPosition() {
        elbowLeftController.setPID(pE, iE, dE);
        elbowLeft.setPower(elbowLeftController.calculate(elbowLeft.getCurrentPosition(), goalElbow));

        elbowRightController.setPID(pE, iE, dE);
        elbowRight.setPower(elbowRightController.calculate(elbowRight.getCurrentPosition(), goalElbow));

        if (armAtElbowTargetPosition()){
            shoulderLeftController.setPIDF(pS, iS, dS,fS);
            shoulderLeft.setPower(shoulderLeftController.calculate(shoulderLeft.getCurrentPosition(), goalShoulder));

            shoulderRightController.setPIDF(pS, iS, dS,fS);
            shoulderRight.setPower(shoulderRightController.calculate(shoulderRight.getCurrentPosition(), goalShoulder));
        }
        else {}

        if (armShoulderAtTargetPosition()){

        } else{}


        // Check if the arm has reached the target position and update the state if needed
        if (armAtTargetPosition()) {
            // Update state or perform other necessary actions
        }
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
    public void increasePosition(int increaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = elbowLeft.getCurrentPosition();
        newPosition = currentPosition + increaseAmount;

    }

    public void decreasePosition(int decreaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = elbowLeft.getCurrentPosition();
        newPosition = currentPosition - decreaseAmount;
        changePosition(newPosition);
    }

    public void changePosition(int newPosition) {
        elbowLeft.setTargetPosition(newPosition);
        elbowLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowLeft.setPower(1.0);
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
    public void incrementWristPosition (boolean buttonpress) {
        double currWristPos = wrist.getPosition();
        newWristposition = currWristPos + WRIST_INCREMENT;
        wrist.setPosition(newWristposition);
    }
    public void decrementWristPosition (boolean buttonpress) {
        double currWristPos = wrist.getPosition();
        newWristposition = currWristPos - WRIST_INCREMENT;
        wrist.setPosition(newWristposition);
    }

    public double getWristPosition(){ return wrist.getPosition();}

    public void setMotorMode(int mode) {
        if (mode == 1) elbowLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (mode == 2) elbowLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder() {
        elbowLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public enum ArmState {
        INIT,
        INIT_SET_SERVO,
        INIT_RAISE_ARM,
        TRANSPORT,
        TRANSPORT_RAISE_ARM,
        TRANSPORT_SET_SERVO,
        LOW_SCORE,
        HIGH_SCORE,
        BACK_LOW_SCORE,
        IDLE
    }
    private final int[] armMoveOrder = {1, 2, 3};
}   //end program
