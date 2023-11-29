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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    int maxPosition = 2000;;

    int liftLevelZero = 0;
    int LiftLowPosition = 560;
    int LiftMidPosition = 1134;
    int LiftHighPosition = 1660;

    HardwareMap hardwareMap =  null;

    /* Constructor */
    public LiftUtil(LinearOpMode opmode){myOpMode = opmode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hardwareMap = ahwMap;
        leftLift = hardwareMap.get(DcMotor.class, "leftlift");
        leftLift.setDirection(DcMotor.Direction.FORWARD);
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
            leftLift.setTargetPosition(liftLevelZero);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setPower(1.0);
            rightLift.setTargetPosition(liftLevelZero);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setPower(1.0);
        }
        else if (positionLevel == 1)
        {
            //also low position
            leftLift.setTargetPosition(LiftLowPosition);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setPower(1.0);
            rightLift.setTargetPosition(LiftLowPosition);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setPower(1.0);
        }
        else if (positionLevel == 2)
        {
            leftLift.setTargetPosition(LiftMidPosition);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setPower(1.0);
            rightLift.setTargetPosition(LiftMidPosition);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setPower(1.0);
        }
        else if (positionLevel == 3)
        {
            leftLift.setTargetPosition(LiftHighPosition);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setPower(1.0);
            rightLift.setTargetPosition(LiftHighPosition);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setPower(1.0);
        }

        else {
        }
        //return;

    }

    public void lockCurrentPosition() {
        int leftCurrentPosition;
        leftCurrentPosition = leftLift.getCurrentPosition();
        leftLift.setTargetPosition(leftCurrentPosition);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setPower(1.0);
        int rightCurrentPosition;
        rightCurrentPosition = rightLift.getCurrentPosition();
        rightLift.setTargetPosition(leftCurrentPosition);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setPower(1.0);
    }

    public void increasePosition(int increaseAmount) {
        int LeftcurrentPosition;
        int LeftnewPosition;
        LeftcurrentPosition = leftLift.getCurrentPosition();
        LeftnewPosition = LeftcurrentPosition + increaseAmount;
        changePosition(LeftnewPosition);

        int RightcurrentPosition;
        int RightnewPosition;
        RightcurrentPosition = rightLift.getCurrentPosition();
        RightnewPosition = RightcurrentPosition + increaseAmount;
        changePosition(RightnewPosition);
    }

    public void decreasePosition(int decreaseAmount) {

        int LeftCurrentPosition;
        int LeftnewPosition;
        LeftCurrentPosition = leftLift.getCurrentPosition();
        LeftnewPosition = LeftCurrentPosition - decreaseAmount;
        changePosition(LeftnewPosition);

        int RightcurrentPosition;
        int RightnewPosition;
        RightcurrentPosition = rightLift.getCurrentPosition();
        RightnewPosition = RightcurrentPosition - decreaseAmount;
        changePosition(RightnewPosition);
    }

    public void changePosition(int newPosition) {
        leftLift.setTargetPosition(newPosition);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setPower(1.0);

        rightLift.setTargetPosition(newPosition);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setPower(1.0);
    }

    public void setMotorPower(double v) {
        if (leftLift.getCurrentPosition() <= maxPosition){
            leftLift.setPower(v);
        } else {
            leftLift.setPower(0);
        }

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
        return leftLift.getPower();
    }

    public int getMotorPosition() {
        return leftLift.getCurrentPosition();
    }

    public void setMotorMode(int mode) {
        if (mode == 1) leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (mode == 2) leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder(){
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}   //end program
