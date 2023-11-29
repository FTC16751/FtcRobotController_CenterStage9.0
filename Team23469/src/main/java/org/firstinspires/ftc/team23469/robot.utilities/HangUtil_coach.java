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


public class HangUtil_coach {
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
    DcMotor changeseverything;
    int liftPosition;
    int minPosition = 0;
    int maxPosition = 1000;;

    int liftLevelZero = 0;
    int LiftLowPosition = 560;
    int LiftMidPosition = 1134;
    int LiftHighPosition = 1660;

    HardwareMap hardwareMap =  null;

    /* Constructor */
    public HangUtil_coach(LinearOpMode opmode){myOpMode = opmode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hardwareMap = ahwMap;
        changeseverything = hardwareMap.get(DcMotor.class, "hanger");
        changeseverything.setDirection(DcMotor.Direction.FORWARD);
        changeseverything.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        changeseverything.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void raiseToPosition(int positionLevel, Double targetSpeed) {
        if (positionLevel == 0)
        {
            //zero
            changeseverything.setTargetPosition(liftLevelZero);
            changeseverything.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            changeseverything.setPower(1.0);
        }
        else if (positionLevel == 1)
        {
            //also low position
            changeseverything.setTargetPosition(LiftLowPosition);
            changeseverything.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            changeseverything.setPower(1.0);
        }
        else if (positionLevel == 2)
        {
            changeseverything.setTargetPosition(LiftMidPosition);
            changeseverything.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            changeseverything.setPower(1.0);
        }
        else if (positionLevel == 3)
        {
            changeseverything.setTargetPosition(LiftHighPosition);
            changeseverything.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            changeseverything.setPower(1.0);
        }

        else {
        }
        //return;

    }

    public void lockCurrentPosition() {
        int leftCurrentPosition;
        leftCurrentPosition = changeseverything.getCurrentPosition();
        changeseverything.setTargetPosition(leftCurrentPosition);
        changeseverything.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        changeseverything.setPower(1.0);
    }

    public void increasePosition(int increaseAmount) {

        int LeftcurrentPosition;
        int LeftnewPosition;
        LeftcurrentPosition = changeseverything.getCurrentPosition();
        LeftnewPosition = LeftcurrentPosition + increaseAmount;
        changePosition(LeftnewPosition);
    }

    public void decreasePosition(int decreaseAmount) {

        int LeftCurrentPosition;
        int LeftnewPosition;
        LeftCurrentPosition = changeseverything.getCurrentPosition();
        LeftnewPosition = LeftCurrentPosition - decreaseAmount;
        changePosition(LeftnewPosition);
    }

    public void changePosition(int newPosition) {
        changeseverything.setTargetPosition(newPosition);
        changeseverything.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        changeseverything.setPower(1.0);
    }

    public void setMotorPower(double v) {
        if (changeseverything.getCurrentPosition() <= maxPosition){
            changeseverything.setPower(v);
        } else {
            changeseverything.setPower(0);
        }
    }

    public void stopArm() {
        setMotorPower(0);
    }//StopArm

    public double getMotorPower() {
        return changeseverything.getPower();
    }

    public int getMotorPosition() {
        return changeseverything.getCurrentPosition();
    }

    public void setMotorMode(int mode) {
        if (mode == 1) changeseverything.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (mode == 2) changeseverything.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder(){
        changeseverything.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}   //end program
