/* P3 2021-22 season Auto Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.teamcode.robot_utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SpinnerUtil {

    //using GoBilda 5202 motor
    static final double COUNTS_PER_MOTOR_REV = 537;  //*2 adjust for mecanum
    static final double WHEEL_DIAMETER = 9.6;     // In centimeters
    static final double WHEEL_RADIUS = WHEEL_DIAMETER/2; // in cm
    static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    static final double GEAR_REDUCTION = .99; // output (wheel) speed / input (motor) speed
    static final double COUNTS_PER_GEAR_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
    static final double COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV/360;

    //Duck spinner powered by gobilda yellowjacket motor
    DcMotor DuckSpinnerMotor;
    double motorPower;

    /* local OpMode members. */
    HardwareMap hardwareMap          =  null;

    /* Constructor */
    public SpinnerUtil(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;

        DuckSpinnerMotor = hardwareMap.get(DcMotor.class, "Shoot");
        DuckSpinnerMotor.setDirection(DcMotor.Direction.FORWARD);
        DuckSpinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DuckSpinnerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorPower = .6;
    }

    public void SpinClockwise(Double targetSpeed) {
        motorPower = (targetSpeed > 0 ) ? targetSpeed : motorPower;
        DuckSpinnerMotor.setPower(motorPower);
    }

    public void SpinCounterClockwise(Double targetSpeed) {
        motorPower = (targetSpeed > 0) ? targetSpeed : motorPower;
        DuckSpinnerMotor.setPower(-motorPower);
    }

    public void setSpinMotorPower(double v) {
        DuckSpinnerMotor.setPower(v);
    }

    public void stopSpinner() {
        DuckSpinnerMotor.setPower(0);
    }//StopSpinner

    public double getMotorPower() { return DuckSpinnerMotor.getPower(); }

}   //end program
