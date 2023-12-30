/* P3 2021-22 season Auto Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.team16751.robot.utilities;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
// p =0.002 i = 0.1 d = 0.0001 f = 0.1

public class ArmUtil_backup {
    public static final double SHOULDER_GOAL_POSITION_TRANSPORT = 300;
    //using GoBuilda 5202 motor
    public static double p = 0.002, i = 0.03, d = 0.0001, f = 0.1;
    private final PIDController shoulderLeftController = new PIDController(p, i, d);
    private final PIDController elbowLeftController = new PIDController(p, i, d);
    private final PIDController shoulderRightController = new PIDController(p, i, d);
    private final PIDController elbowRightController = new PIDController(p, i, d);
    private final int currentState = 0;
    //Bela delivery arm Definitions
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
    private ArmState currentArmState;

    /* Constructor */
    public ArmUtil_backup(LinearOpMode opmode) {
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
        wrist.setPosition(0.25);

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
                currentArmState = ArmState.INIT;
                raiseToPosition(1, .5);
                break;
            case TRANSPORT:
                currentArmState = ArmState.TRANSPORT;
                raiseToPosition(2, .5);
                break;
            case LOW_SCORE:
                currentArmState = ArmState.LOW_SCORE;
                raiseToPosition(3, .5);
                break;
            case HIGH_SCORE:
                currentArmState = ArmState.HIGH_SCORE;
                raiseToPosition(4, .5);
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
            wrist.setPosition(0.25);
        } else if (positionLevel == 2) //acquire
        {
            goalShoulder = SHOULDER_GOAL_POSITION_TRANSPORT;
            goalElbow = 0;
            moveArmToPosition();
        } else if (positionLevel == 3) //score low
        {
            goalShoulder = 1250;
            goalElbow = 0;
            moveArmToPosition();
            wrist.setPosition(0.25);
        } else if (positionLevel == 4) //transport
        {
            goalShoulder = 1650;
            goalElbow = 0;
            moveArmToPosition();
        } else {
            //zero again!
        }
    } //end raise to position

    public void moveArmToPosition() {
        shoulderLeftController.setPID(p, i, d);
        shoulderLeft.setPower(shoulderLeftController.calculate(shoulderLeft.getCurrentPosition(), goalShoulder));

        shoulderRightController.setPID(p, i, d);
        shoulderRight.setPower(shoulderRightController.calculate(shoulderRight.getCurrentPosition(), goalShoulder));

        elbowLeftController.setPID(p, i, d);
        elbowLeft.setPower(elbowLeftController.calculate(elbowLeft.getCurrentPosition(), goalElbow));

        elbowRightController.setPID(p, i, d);
        elbowRight.setPower(elbowRightController.calculate(elbowRight.getCurrentPosition(), goalElbow));

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

    public int getelbowMotorPosition() {
        return elbowLeft.getCurrentPosition();
    }

    public int getshoulderMotorPosition() {
        return shoulderLeft.getCurrentPosition();
    }

    public void setMotorMode(int mode) {
        if (mode == 1) elbowLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (mode == 2) elbowLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder() {
        elbowLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public enum ArmState {
        INIT,
        TRANSPORT,
        LOW_SCORE,
        HIGH_SCORE
    }
}   //end program
