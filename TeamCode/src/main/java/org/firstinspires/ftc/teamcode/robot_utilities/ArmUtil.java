/* P3 2021-22 season Auto Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.teamcode.robot_utilities;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
// p =0.002 i = 0.1 d = 0.0001 f = 0.1

public class ArmUtil {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;

    //using GoBuilda 5202 motor
    public static double p=0.002, i=0.1,d=0.0001,f=0.1;
    //Bela delivery arm Definitions
    public double goalShoulder;
    public double goalElbow;
    private PIDController shoulderController = new PIDController(p,i,d);
    private PIDController elbowController = new PIDController(p,i,d);

    DcMotor elbow;
    public double currElbow = elbow.getCurrentPosition();

    DcMotor shoulder;
    public double currShoulder = shoulder.getCurrentPosition();
    Servo wrist;


    //The following below reads the positions for the elbow joint (joint #2)
    int elbowTransport = 0;//POSITION 4


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

        wrist = hardwareMap.get(Servo.class, "Wrist");
        wrist.setDirection(Servo.Direction.FORWARD);
        wrist.setPosition(0.25);
    }




    public void raiseToPosition(int positionLevel, Double targetSpeed) {
        if (positionLevel == 1) //rest
        {
            goalElbow = 0;
            elbowController.setPID(p,i,d);
            elbow.setPower(elbowController.calculate(currElbow,goalElbow));


            goalShoulder = 0;
            shoulderController.setPID(p,i,d);
            shoulder.setPower(shoulderController.calculate(currShoulder,goalShoulder));


            wrist.setPosition(0.25);

        }
        else if (positionLevel == 2) //acquire
        {
            goalElbow = -1000;
            elbowController.setPID(p,i,d);
            elbow.setPower(elbowController.calculate(currElbow,goalElbow));


            goalShoulder = -1000;
            shoulderController.setPID(p,i,d);
            shoulder.setPower(shoulderController.calculate(currShoulder,goalShoulder));
        }
        else if (positionLevel == 3) //score low
        {
            goalElbow = -700;
            elbowController.setPID(p,i,d);
            elbow.setPower(elbowController.calculate(currElbow,goalElbow));


            goalShoulder = -500;
            shoulderController.setPID(p,i,d);
            shoulder.setPower(shoulderController.calculate(currShoulder,goalShoulder));

            wrist.setPosition(0.25);

        }
      else if (positionLevel == 4) //transport
        {
            elbow.setTargetPosition(elbowTransport);
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbow.setPower(1.0);
            wrist.setPosition(0.25);
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
    public void setElbowMotorPower(double v) {
        elbow.setPower(v);
    }
    public void setShoulderMotorPower(double v) {
        shoulder.setPower(v);
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

    public int getelbowMotorPosition() {
        return elbow.getCurrentPosition();
    }
    public int getshoulderMotorPosition() {
        return shoulder.getCurrentPosition();
    }
    public void setMotorMode(int mode) {
        if (mode == 1) elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (mode == 2) elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder(){
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}   //end program
