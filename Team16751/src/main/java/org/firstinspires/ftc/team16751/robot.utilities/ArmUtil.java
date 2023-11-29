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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ArmUtil {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;

    //using GoBuilda 5202 motor
    static final double COUNTS_PER_MOTOR_REV = 537;
    static final double GEAR_REDUCTION = 1.00; // output (wheel) speed / input (motor) speed

    //Bela delivery arm Definitions
    DcMotor elbow;
    int armPosition;
    int minPosition = 0;
    int maxPosition = 1000;;
    boolean isarmBusy = false;


    DcMotor shoulder;
    int shoulderPosition;
    int shoulderMin = 0;
    int shoulderMax = 1000;;



    //The following below reads the positions for the elbow joint (joint #2)
    int elbowRest = (int)(-2); //POSITION 1
    int shoulderRest = (int)(0);
    int elbowAcquire = 19; //POSITION 2
    int shoulderAcquire = 104;
    int elbowLowScore = 100;//125;//POSITION 3
    int shoulderLowScore = 82;
    int elbowTransport = 0;//POSITION 4

    enum ArmState {
        START,
        ACQUIRE,
        LOWSCORE,
        TRANSPORT,
    }




            /* local OpMode members. */
    HardwareMap hardwareMap =  null;

    /* Constructor */
    public ArmUtil(LinearOpMode opmode){myOpMode = opmode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;
        elbow = hardwareMap.get(DcMotor.class, "Elbow");
        elbow.setDirection(DcMotor.Direction.FORWARD);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shoulder = hardwareMap.get(DcMotor.class, "Shoulder");
        shoulder.setDirection(DcMotor.Direction.FORWARD);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void raiseToPosition(int positionLevel, Double targetSpeed) {
        if (positionLevel == 1)
        {
            //also zero
            elbow.setTargetPosition(elbowRest);
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbow.setPower(0.5);


            shoulder.setTargetPosition(shoulderRest);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(0.5);

        }
        else if (positionLevel == 2)
        {
            elbow.setTargetPosition(elbowAcquire);
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbow.setPower(0.5);


            shoulder.setTargetPosition(shoulderAcquire);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(0.5);


        }
        else if (positionLevel == 3)
        {
            elbow.setTargetPosition(elbowLowScore);
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbow.setPower(0.5);

            shoulder.setTargetPosition(shoulderLowScore);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(0.5);



        }
      else if (positionLevel == 4)
        {
            elbow.setTargetPosition(elbowTransport);
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbow.setPower(1.0);

        }
        else {
            //zero again!
            /*
            armPosition =(int)(0);
            elbow.setTargetPosition(armPosition);
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbow.setPower(0.5);

             */



        }
        //return;
    }

    public void lockCurrentPosition() {
        int currentPosition;
        currentPosition = elbow.getCurrentPosition();
        int currentPositionShoulder;

        currentPositionShoulder = shoulder.getCurrentPosition();
        elbow.setTargetPosition(currentPosition);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setPower(1.0);
        shoulder.setTargetPosition(currentPositionShoulder);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(0.5);
    }

    public void increasePosition(int increaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = elbow.getCurrentPosition();
        newPosition = currentPosition + increaseAmount;

    }

    public void decreasePosition(int decreaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = elbow.getCurrentPosition();
        newPosition = currentPosition - decreaseAmount;
        changePosition(newPosition);
    }

    public void changePosition(int newPosition) {
        elbow.setTargetPosition(newPosition);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setPower(1.0);
    }

    public void setMotorPower(double v) {
            elbow.setPower(v);
    }

    public void stopArm() {
        setMotorPower(0);
    }//StopArm

    public double getMotorPower() {
        return elbow.getPower();
    }

    public int getMotorPosition() {
        return elbow.getCurrentPosition();
    }

    public void setMotorMode(int mode) {
        if (mode == 1) elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (mode == 2) elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder(){
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}   //end program
