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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

public class GenericMotorUtil {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;

    //drive train variables
    private DcMotor motor1;
    static double   DRIVE_SPEED = 1;     // Nominal speed for better accuracy.;

    /* local OpMode members. */
    HardwareMap hardwareMap          =  null;

    /* Constructor */
    public GenericMotorUtil(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hardwareMap = ahwMap;

        // Initialize the chassis drive motors.
        motor1 = hardwareMap.get(DcMotor.class, "Elbow");

        // Reverse left motors for driving (so it goes counterclockwise to drive forward)
        motor1.setDirection(DcMotor.Direction.REVERSE);

        //Set brake condition to full stop (allso can give DcMotor.ZeroPowerBehavior.FLOAT)
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set Drive Speed
        DRIVE_SPEED = 1;    //1=Full Speed

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void stopMotors() {
        motor1.setPower(0);
    }

    public void setMotorPowers(double m1) {
        motor1.setPower(m1);
    }



}   //end program
