/* P3 2021-22 season Auto Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.teamcode.robot.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class SuzanUtil {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;
    LiftUtil lift = new LiftUtil(null);

    //using GoBilda 5202 motor
    static final double COUNTS_PER_MOTOR_REV = 1425.1;
    static final double GEAR_REDUCTION = 1.00; // output (wheel) speed / input (motor) speed
    static final double COUNTS_PER_GEAR_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
    static final double COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV/360;

    //alia lazy suzan but not a lazy suzan but a worm gear Definitions
    DcMotor SuzanMotor;
    int armPosition;
    int startPosition = 0;
    int minPosition = -1050;
    int maxPosition = (int)(1200);

    int leftPosition = -1030;
    int rightPosition = 1165;

    /* local OpMode members. */
    static HardwareMap hardwareMap          =  null;

    /* Constructor */
    public SuzanUtil(LinearOpMode opmode){myOpMode = opmode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;
        //import the hardware configuration for the turret and set drive modes
        SuzanMotor = hardwareMap.get(DcMotor.class, "lazysuzan");
        SuzanMotor.setDirection(DcMotor.Direction.FORWARD);
        SuzanMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SuzanMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isSafetoRotate(){
        int safePosition = 400;
        int liftPosition = lift.getMotorPosition();
        if (liftPosition >= safePosition) return true;
        else return false;
    }
    public boolean turretIsBusy(){
       return SuzanMotor.isBusy();
    }

    public int rotate (int position, Double targetSpeed) {

            SuzanMotor.setTargetPosition(position);
            SuzanMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SuzanMotor.setPower(targetSpeed);
            return position;
    }

    public void lockCurrentPosition() {
        int currentPosition;
        currentPosition = SuzanMotor.getCurrentPosition();
        SuzanMotor.setTargetPosition(currentPosition);
        SuzanMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SuzanMotor.setPower(1.0);
    }

    public void increasePosition(int increaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = SuzanMotor.getCurrentPosition();
        newPosition = currentPosition + increaseAmount;
        changePosition(newPosition);
    }

    public void decreasePosition(int decreaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = SuzanMotor.getCurrentPosition();
        newPosition = currentPosition - decreaseAmount;
        changePosition(newPosition);
    }

    public int changePosition(int newPosition) {
        SuzanMotor.setTargetPosition(newPosition);
        SuzanMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SuzanMotor.setPower(.25);
        return newPosition;
    }

    public void setMotorPower(double v) {
        SuzanMotor.setPower(v);
    }

    public void stopMotor() {
        setMotorPower(0);
    }//StopArm

    public double getMotorPower() {
        return SuzanMotor.getPower();
    }

    public int getMotorPosition() {
        return SuzanMotor.getCurrentPosition();
    }

    public void setMotorMode(int mode) {
        if (mode == 1) SuzanMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (mode == 2) SuzanMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder(){
        SuzanMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}   //end program
