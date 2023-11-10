/* P3 2021-22 season Auto Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.team23469.robot.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class LiftUtil {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;

    //using GoBilda 5202 motor
    static final double COUNTS_PER_MOTOR_REV = 537;
    static final double WHEEL_DIAMETER = 9.6;     // In centimeters
    static final double GEAR_REDUCTION = 1.00; // output (wheel) speed / input (motor) speed
    static final double COUNTS_PER_GEAR_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
    static final double COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV/360;
    static final double WHEEL_RADIUS = WHEEL_DIAMETER/2; // in cm
    static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    static final double groundOffset = 6.0;

    //Bela delivery arm Definitions
    DcMotor rightLift;
    DcMotor leftLift;
    int liftPosition;
    int minPosition = 0;
    int maxPosition = 1000;;

    int liftLevelZero = 0;
    int LiftLowPosition = 1785;
    int LiftMidPosition = 3130;
    int LiftHighPosition = 4262;

    HardwareMap hardwareMap =  null;

    /* Constructor */
    public LiftUtil(LinearOpMode opmode){myOpMode = opmode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hardwareMap = ahwMap;
        leftLift = hardwareMap.get(DcMotor.class, "leftlift");
        leftLift.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLift = hardwareMap.get(DcMotor.class, "rightlift");
        rightLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void raiseToPosition(int positionLevel, Double targetSpeed) {
        if (positionLevel == 0)
        {
            //zero
            rightLift.setTargetPosition(liftLevelZero);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setPower(1.0);
        }
        else if (positionLevel == 1)
        {
            //also low position
            rightLift.setTargetPosition(LiftLowPosition);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setPower(1.0);
        }
        else if (positionLevel == 2)
        {
            rightLift.setTargetPosition(LiftMidPosition);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setPower(1.0);
        }
        else if (positionLevel == 3)
        {
            rightLift.setTargetPosition(LiftHighPosition);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setPower(1.0);
        }

        else {
        }
        //return;

    }

    public void lockCurrentPosition() {
        int currentPosition;
        currentPosition = rightLift.getCurrentPosition();
        rightLift.setTargetPosition(currentPosition);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setPower(1.0);
    }

    public void increasePosition(int increaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = rightLift.getCurrentPosition();
        newPosition = currentPosition + increaseAmount;
        changePosition(newPosition);
    }

    public void decreasePosition(int decreaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = rightLift.getCurrentPosition();
        newPosition = currentPosition - decreaseAmount;
        changePosition(newPosition);
    }

    public void changePosition(int newPosition) {
        rightLift.setTargetPosition(newPosition);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setPower(1.0);
    }

    public void setMotorPower(double v) {
        if (rightLift.getCurrentPosition() <= maxPosition){
            rightLift.setPower(v);
        } else {
            rightLift.setPower(0);
        }
    }

    public void stopArm() {
        setMotorPower(0);
    }//StopArm

    public double getMotorPower() {
        return rightLift.getPower();
    }

    public int getMotorPosition() {
        return rightLift.getCurrentPosition();
    }

    public void setMotorMode(int mode) {
        if (mode == 1) rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (mode == 2) rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder(){
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}   //end program
