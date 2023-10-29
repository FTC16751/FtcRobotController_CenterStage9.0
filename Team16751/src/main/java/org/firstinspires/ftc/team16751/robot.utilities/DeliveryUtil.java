/* P3 2021-22 season Auto Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.team16751.robot.utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DeliveryUtil {

    //using GoBilda 5202 motor
    static final double COUNTS_PER_MOTOR_REV = 1425.1;
    static final double GEAR_REDUCTION = 1.00; // output (wheel) speed / input (motor) speed
    static final double COUNTS_PER_GEAR_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
    static final double COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV/360;

    //Bela delivery arm Definitions
    DcMotor belaArm;
    int armPosition;
    int minPosition = 0;
    int maxPosition = (int)(COUNTS_PER_DEGREE * 270);;

    int armLevel1Position = (int)(COUNTS_PER_DEGREE * 0);
    int armLevel2Position = (int)(COUNTS_PER_DEGREE * 50);
    int armLevel3Position = (int)(COUNTS_PER_DEGREE * 70);
    int armLevel4Position = (int)(COUNTS_PER_DEGREE * 90);

    //int armLevel3Position = (int)(COUNTS_PER_DEGREE * 235);

    /* local OpMode members. */
    HardwareMap hardwareMap          =  null;

    /* Constructor */
    public DeliveryUtil(){
    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;

        //Billy Arm
        belaArm = hardwareMap.get(DcMotor.class, "billyarm");
        belaArm.setDirection(DcMotor.Direction.REVERSE);
        belaArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        belaArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public double raiseToPosition(int positionLevel, Double targetSpeed) {
        if (positionLevel == 0 && belaArm.getCurrentPosition() < maxPosition)
        {
            belaArm.setTargetPosition((int)(1));
            belaArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            belaArm.setPower(.8);
            return positionLevel;
        }
        else if (positionLevel == 1 && belaArm.getCurrentPosition() < maxPosition)
        {
            belaArm.setTargetPosition((int)(armLevel1Position));
            belaArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            belaArm.setPower(0.8);
            return positionLevel;
        }
        else if (positionLevel == 2 && belaArm.getCurrentPosition() < maxPosition)
        {
            belaArm.setTargetPosition(armLevel2Position);
            belaArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            belaArm.setPower(0.8);
        }
        else if (positionLevel == 3 && belaArm.getCurrentPosition() < maxPosition)
        {
            belaArm.setTargetPosition(armLevel3Position);
            belaArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            belaArm.setPower(0.8);
        }
        else if (positionLevel == 4 && belaArm.getCurrentPosition() < maxPosition)
        {
            belaArm.setTargetPosition(armLevel4Position);
            belaArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            belaArm.setPower(0.8);
        }
        else {
            armPosition =(int)(0);
            belaArm.setTargetPosition(armPosition);
            belaArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            belaArm.setPower(0.8);
        }

        return 0;
    }

    public void lockCurrentPosition() {
        int currentPosition;
        currentPosition = belaArm.getCurrentPosition();
        belaArm.setTargetPosition(currentPosition);
        belaArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        belaArm.setPower(1.0);
    }

    public void increasePosition(int increaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = belaArm.getCurrentPosition();
        newPosition = currentPosition + increaseAmount;
        changePosition(newPosition);
    }

    public void decreasePosition(int decreaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = belaArm.getCurrentPosition();
        newPosition = currentPosition - decreaseAmount;
        changePosition(newPosition);
    }

    public void changePosition(int newPosition) {
        belaArm.setTargetPosition(newPosition);
        belaArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        belaArm.setPower(0.7);
    }

    public void setMotorPower(double v) {
        belaArm.setPower(v);
    }

    public void stopArm() {
        setMotorPower(0);
    }//StopArm

    public double getMotorPower() {
        return belaArm.getPower();
    }

    public int getMotorPosition() {
        return belaArm.getCurrentPosition();
    }

    public void setMotorMode(int mode) {
        if (mode == 1) belaArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (mode == 2) belaArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder(){
        belaArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}   //end program
