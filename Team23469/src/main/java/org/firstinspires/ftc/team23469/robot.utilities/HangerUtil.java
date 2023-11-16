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


public class HangerUtil {
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
    DcMotor hanger;
    int liftPosition;
    int minPosition = 0;
    int maxPosition = 1000;;

    int hangerLevelZero = 0;
    int hangerLowPosition = 6000;
    int hangerMidPosition = 12000;

    HardwareMap hardwareMap =  null;

    /* Constructor */
    public HangerUtil(LinearOpMode opmode){myOpMode = opmode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hardwareMap = ahwMap;
        hanger = hardwareMap.get(DcMotor.class, "hanger");
        hanger.setDirection(DcMotor.Direction.FORWARD);
        hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void raiseToPosition(int positionLevel, Double targetSpeed) {
        if (positionLevel == 0)
        {
            //zero
            hanger.setTargetPosition(hangerLevelZero);
            hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hanger.setPower(1.0);
        }
        else if (positionLevel == 1)
        {
            //also low position
            hanger.setTargetPosition(hangerLowPosition);
            hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hanger.setPower(1.0);
        }
        else if (positionLevel == 2)
        {
            hanger.setTargetPosition(hangerMidPosition);
            hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hanger.setPower(1.0);
        }

        else {
        }
        //return;

    }

    public void lockCurrentPosition() {
        int leftCurrentPosition;
        leftCurrentPosition = hanger.getCurrentPosition();
        hanger.setTargetPosition(leftCurrentPosition);
        hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hanger.setPower(1.0);
    }

    public void increasePosition(int increaseAmount) {

        int LeftcurrentPosition;
        int LeftnewPosition;
        LeftcurrentPosition = hanger.getCurrentPosition();
        LeftnewPosition = LeftcurrentPosition + increaseAmount;
        changePosition(LeftnewPosition);
    }

    public void decreasePosition(int decreaseAmount) {

        int LeftCurrentPosition;
        int LeftnewPosition;
        LeftCurrentPosition = hanger.getCurrentPosition();
        LeftnewPosition = LeftCurrentPosition - decreaseAmount;
        changePosition(LeftnewPosition);
    }

    public void changePosition(int newPosition) {
        hanger.setTargetPosition(newPosition);
        hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hanger.setPower(1.0);
    }

    public void setMotorPower(double v) {
        if (hanger.getCurrentPosition() <= maxPosition){
            hanger.setPower(v);
        } else {
            hanger.setPower(0);
        }
    }

    public void stopArm() {
        setMotorPower(0);
    }//StopArm

    public double getMotorPower() {
        return hanger.getPower();
    }

    public int getMotorPosition() {
        return hanger.getCurrentPosition();
    }

    public void setMotorMode(int mode) {
        if (mode == 1) hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (mode == 2) hanger.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder(){
        hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}   //end program
