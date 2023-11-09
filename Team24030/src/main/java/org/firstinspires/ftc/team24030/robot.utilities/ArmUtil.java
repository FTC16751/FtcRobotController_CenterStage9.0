/* P3 2021-22 season Auto Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.team24030.robot.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ArmUtil {

    private LinearOpMode myOpMode = null;

    //using GoBilda 5202 motor
    static final double COUNTS_PER_MOTOR_REV = 1425.1;
    static final double GEAR_REDUCTION = 1.00; // output (wheel) speed / input (motor) speed
    static final double COUNTS_PER_GEAR_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
    static final double COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV/360;

    //Bela delivery arm Definitions
    DcMotor shoulderArm;
    int armPosition;
    int minPosition = 0;
    int maxPosition = (int)(COUNTS_PER_DEGREE * 270);;

    int armLevel1Position = (int)(0);
    int armLevel2Position = (int)(404);
    int armLevel3Position = (int)(COUNTS_PER_DEGREE * 70);
    int armLevel4Position = (int)(COUNTS_PER_DEGREE * 90);

    /* local OpMode members. */
    HardwareMap hardwareMap          =  null;

    /* Constructor */
    public ArmUtil(LinearOpMode opmode) {
        myOpMode = opmode;
    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;

        shoulderArm = hardwareMap.get(DcMotor.class, "Shoulder");
        shoulderArm.setDirection(DcMotor.Direction.FORWARD);
        shoulderArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void raiseToPosition(int positionLevel, Double targetSpeed) {
        if (positionLevel == 0 && shoulderArm.getCurrentPosition() < maxPosition)
        {
            shoulderArm.setTargetPosition((int)(1));
            shoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderArm.setPower(.8);
        }
        else if (positionLevel == 1 && shoulderArm.getCurrentPosition() < maxPosition)
        {
            shoulderArm.setTargetPosition((int)(armLevel1Position));
            shoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderArm.setPower(0.8);
        }
        else if (positionLevel == 2 && shoulderArm.getCurrentPosition() < maxPosition)
        {
            shoulderArm.setTargetPosition(armLevel2Position);
            shoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderArm.setPower(0.8);
        }
        else if (positionLevel == 3 && shoulderArm.getCurrentPosition() < maxPosition)
        {
            shoulderArm.setTargetPosition(armLevel3Position);
            shoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderArm.setPower(0.8);
        }
        else if (positionLevel == 4 && shoulderArm.getCurrentPosition() < maxPosition)
        {
            shoulderArm.setTargetPosition(armLevel4Position);
            shoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderArm.setPower(0.8);
        }
        else {
            armPosition =(int)(0);
            shoulderArm.setTargetPosition(armPosition);
            shoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderArm.setPower(0.8);
        }

    }

    public void lockCurrentPosition() {
        int currentPosition;
        currentPosition = shoulderArm.getCurrentPosition();
        shoulderArm.setTargetPosition(currentPosition);
        shoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderArm.setPower(1.0);
    }

    public void increasePosition(int increaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = shoulderArm.getCurrentPosition();
        newPosition = currentPosition + increaseAmount;
        changePosition(newPosition);
    }

    public void decreasePosition(int decreaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = shoulderArm.getCurrentPosition();
        newPosition = currentPosition - decreaseAmount;
        changePosition(newPosition);
    }

    public void changePosition(int newPosition) {
        shoulderArm.setTargetPosition(newPosition);
        shoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderArm.setPower(0.7);
    }

    public void setMotorPower(double v) {
        shoulderArm.setPower(v);
    }

    public void stopArm() {
        setMotorPower(0);
    }//StopArm

    public double getMotorPower() {
        return shoulderArm.getPower();
    }

    public int getMotorPosition() {
        return shoulderArm.getCurrentPosition();
    }

    public void setMotorMode(int mode) {
        if (mode == 1) shoulderArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (mode == 2) shoulderArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder(){
        shoulderArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}   //end program
