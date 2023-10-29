/* P3 2021-22 season Auto Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.team16751.robot.utilities;

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
    DcMotor belaLift;
    int liftPosition;
    int minPosition = 0;
    int maxPosition = 1000;;
    boolean isliftBusy = false;

    int liftLevelZero = (int)(0);


    /* the following settings are for the main robot #1 which has a slower lifte motor at the current time */

    int LiftLowPosition = 1785;
    int LiftMidPosition = 3130;
    int LiftHighPosition = 4262;


    // use the following settings for the faster lift motor which is being tested on bot #2 */
    //int LiftLowPosition  = 1231;
    //int LiftMidPosition  = 2012;
    //int LiftHighPosition = 2900;



    /* local OpMode members. */
    HardwareMap hardwareMap =  null;

    /* Constructor */
    public LiftUtil(LinearOpMode opmode){myOpMode = opmode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;
        //Billy Arm is now bela lift
        belaLift = hardwareMap.get(DcMotor.class, "billyarm");
        belaLift.setDirection(DcMotor.Direction.REVERSE);
        belaLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        belaLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public boolean raiseToPosition(int positionLevel, Double targetSpeed) {
        if (positionLevel == 0)
        {
            //zero
            belaLift.setTargetPosition((int)(0));
            belaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            belaLift.setPower(1.0);
            return belaLift.isBusy();
        }
        else if (positionLevel == 1)
        {
            //also zero
            belaLift.setTargetPosition(liftLevelZero);
            belaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            belaLift.setPower(1.0);
            return belaLift.isBusy();
        }
        else if (positionLevel == 2)
        {
            belaLift.setTargetPosition(LiftLowPosition);
            belaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            belaLift.setPower(1.0);
            return belaLift.isBusy();

        }
        else if (positionLevel == 3)
        {
            belaLift.setTargetPosition(LiftMidPosition);
            belaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            belaLift.setPower(1.0);
            return belaLift.isBusy();
        }
        else if (positionLevel == 4)
        {
            belaLift.setTargetPosition(LiftHighPosition);
            belaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            belaLift.setPower(1.0);
            return belaLift.isBusy();
        }
        else if (positionLevel == 5) {
            belaLift.setTargetPosition(323);
            belaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            belaLift.setPower(1.0);
            return belaLift.isBusy();
        }
        else if (positionLevel == 6) {
            belaLift.setTargetPosition(250);
            belaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            belaLift.setPower(1.0);
            return belaLift.isBusy();
        }
        else if (positionLevel == 7) {
            belaLift.setTargetPosition(141);
            belaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            belaLift.setPower(1.0);
            return belaLift.isBusy();
        }
        else if (positionLevel == 8) {
            belaLift.setTargetPosition(58);
            belaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            belaLift.setPower(1.0);
            return belaLift.isBusy();
        }
        else {
            //zero again!
            liftPosition =(int)(0);
            belaLift.setTargetPosition(liftPosition);
            belaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            belaLift.setPower(0.8);
            return belaLift.isBusy();
        }
        //return;
    }

    public void lockCurrentPosition() {
        int currentPosition;
        currentPosition = belaLift.getCurrentPosition();
        belaLift.setTargetPosition(currentPosition);
        belaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        belaLift.setPower(1.0);
    }

    public void increasePosition(int increaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = belaLift.getCurrentPosition();
        newPosition = currentPosition + increaseAmount;
        changePosition(newPosition);
    }

    public void decreasePosition(int decreaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = belaLift.getCurrentPosition();
        newPosition = currentPosition - decreaseAmount;
        changePosition(newPosition);
    }

    public void changePosition(int newPosition) {
        belaLift.setTargetPosition(newPosition);
        belaLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        belaLift.setPower(1.0);
    }

    public void setMotorPower(double v) {
        if (belaLift.getCurrentPosition() <= maxPosition){
            belaLift.setPower(v);
        } else {
            belaLift.setPower(0);
        }
    }

    public void stopArm() {
        setMotorPower(0);
    }//StopArm

    public double getMotorPower() {
        return belaLift.getPower();
    }

    public int getMotorPosition() {
        return belaLift.getCurrentPosition();
    }

    public void setMotorMode(int mode) {
        if (mode == 1) belaLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (mode == 2) belaLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder(){
        belaLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}   //end program
