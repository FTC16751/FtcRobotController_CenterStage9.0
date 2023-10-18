/* P3 2021-22 season Auto Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.teamcode.robot.utilities;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class DriveUtil2023 {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;

    //drive train variables
    double LEFT_FRONT_POWER;
    double RIGHT_FRONT_POWER;
    double LEFT_REAR_POWER;
    double RIGHT_REAR_POWER;
    double L_HYPOTENUSE;
    double R_HYPOTENUSE;
    //drive train variables
    private DcMotor left_front_motor;
    private DcMotor right_front_motor;
    private DcMotor left_rear_motor;
    private DcMotor right_rear_motor;
    static double   DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.;

    //using GoBilda 5202 motor
    static final double COUNTS_PER_MOTOR_REV = 537;  //*2 adjust for mecanum
    static final double WHEEL_DIAMETER = 9.6;     // In centimeters
    static final double WHEEL_RADIUS = WHEEL_DIAMETER/2; // in cm
    static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    static final double GEAR_REDUCTION = 1.0; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 16.45; // in
    static final double COUNTS_PER_GEAR_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
    static final double COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV/360;

    public IMU imu;

    private ElapsedTime period  = new ElapsedTime();

    /* local OpMode members. */
    HardwareMap hardwareMap          =  null;

    /* Constructor */
    public DriveUtil2023(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hardwareMap = ahwMap;

        // Initialize the chassis drive motors.
        left_front_motor  = hardwareMap.get(DcMotor.class, "Front_Left");
        right_front_motor = hardwareMap.get(DcMotor.class, "Front_Right");
        left_rear_motor  = hardwareMap.get(DcMotor.class, "Rear_Left");
        right_rear_motor = hardwareMap.get(DcMotor.class, "Rear_Right");

        // Reverse left motors for driving (so it goes counterclockwise to drive forward)
        left_front_motor.setDirection(DcMotor.Direction.REVERSE);
        right_front_motor.setDirection(DcMotor.Direction.FORWARD);
        left_rear_motor.setDirection(DcMotor.Direction.REVERSE);
        right_rear_motor.setDirection(DcMotor.Direction.FORWARD);

        //Set brake condition to full stop (allso can give DcMotor.ZeroPowerBehavior.FLOAT)
        left_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_rear_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_rear_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set Drive Speed
        DRIVE_SPEED = 1;    //1=Full Speed

        // Retrieve and initialize the IMU.
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        /*
        imu = hardwareMap.get(BHI260IMU.class, "imu");
         
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        */
         
        /*
        BHI260IMU.Parameters parameters = new IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
*/
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        //imu.initialize();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }


    /***
     * Routine that will stop robot by setting all motors to 0
     */
    public void stopRobot() {
        // Send calculated power to wheels

        left_front_motor.setPower(0);
        right_front_motor.setPower(0);
        left_rear_motor.setPower(0);
        right_rear_motor.setPower(0);
    }//StopRobot

    /***
     * Routine that drives forward at a given speed
     * @param targetSpeed
     */
    public void driveRobotForward(double targetSpeed) {
        // Send calculated power to wheels
        left_front_motor.setPower(targetSpeed);
        right_front_motor.setPower(targetSpeed);
        left_rear_motor.setPower(targetSpeed);
        right_rear_motor.setPower(targetSpeed);
    }

    /***
     * Routine that drives Backward at a given speed
     * @param targetSpeed
     */
    public void driveRobotBackward(double targetSpeed) {
        // Send calculated power to wheels
        left_front_motor.setPower(-targetSpeed);
        right_front_motor.setPower(-targetSpeed);
        left_rear_motor.setPower(-targetSpeed);
        right_rear_motor.setPower(-targetSpeed);
    }

    /***
     * Routine that strafes right at a given speed
     * @param targetSpeed
     */
    public void driveRobotStrafeRight(double targetSpeed) {
        // Send calculated power to wheels
        left_front_motor.setPower(targetSpeed);
        right_front_motor.setPower(-targetSpeed);
        left_rear_motor.setPower(-targetSpeed);
        right_rear_motor.setPower(targetSpeed);
    }

    /***
     * Routine that strafes left at a given speed
     * @param targetSpeed
     */
    public void driveRobotStrafeLeft(double targetSpeed) {
        // Send calculated power to wheels
        left_front_motor.setPower(-targetSpeed);
        right_front_motor.setPower(targetSpeed);
        left_rear_motor.setPower(targetSpeed);
        right_rear_motor.setPower(-targetSpeed);
    }

    /***
     * Routine to drive a robot forward a certain distance and stop
     * @param distanceInCM
     * @param targetSpeed
     */
    public void driveRobotDistanceForward(double distanceInCM, double targetSpeed) {
        int targetCount;

        //convert sentimeters to number cycles to drive
        //counts_per_rotation/circumference*distane
        targetCount = (int) Math.round(COUNTS_PER_GEAR_REV / WHEEL_CIRCUMFERENCE * distanceInCM);

        //ensure full stop and reset motors to begin counting movement
        stopRobot();
        sleep(10);
        left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target stop and mode for running to a position
        left_front_motor.setTargetPosition(targetCount);
        right_front_motor.setTargetPosition(targetCount);
        left_rear_motor.setTargetPosition(targetCount);
        right_rear_motor.setTargetPosition(targetCount);

        left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //conitnue driving until all of the motors hits the distance
        while (left_front_motor.isBusy() || right_front_motor.isBusy() ) {
            // Send calculated power to wheels
            left_front_motor.setPower(targetSpeed);
            right_front_motor.setPower(targetSpeed);
            left_rear_motor.setPower(targetSpeed);
            right_rear_motor.setPower(targetSpeed);
        }//end while

        stopRobot();
        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }  //end drive robot distance forward

    /***
     * Routine to drive a robot forward a certain distance and stop
     * @param distanceInCM
     * @param targetSpeed
     */
    public void driveRobotDistanceBackward(double distanceInCM, double targetSpeed) {
        int targetCount;

        //convert sentimeters to number cycles to drive
        //counts_per_rotation/circumference*distane
        targetCount = (int) Math.round(COUNTS_PER_GEAR_REV/ WHEEL_CIRCUMFERENCE * distanceInCM);

        //ensure full stop and reset motors to begin counting movement
        stopRobot();
        sleep(10);
        left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target stop and mode for running to a position
        left_front_motor.setTargetPosition(-targetCount);
        right_front_motor.setTargetPosition(-targetCount);
        left_rear_motor.setTargetPosition(-targetCount);
        right_rear_motor.setTargetPosition(-targetCount);

        left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //conitnue driving until all of the motors hits the distance
        while (left_front_motor.isBusy() || right_front_motor.isBusy() || left_rear_motor.isBusy()
                || right_rear_motor.isBusy()) {
            // Send calculated power to wheels
            left_front_motor.setPower(targetSpeed);
            right_front_motor.setPower(targetSpeed);
            left_rear_motor.setPower(targetSpeed);
            right_rear_motor.setPower(targetSpeed);
        }//end while

        stopRobot();
        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }  //end drive robot distance backward

    /***
     * Routine to drive a robot forward a certain distance and stop
     * @param distanceInCM
     * @param targetSpeed
     */
    public void driveRobotDistanceStrafeRight(double distanceInCM, double targetSpeed) {
        int targetCount;

        //convert sentimeters to number cycles to drive
        //counts_per_rotation/circumference*distane
        targetCount = (int) Math.round(COUNTS_PER_GEAR_REV*1.1 / WHEEL_CIRCUMFERENCE * distanceInCM);

        //ensure full stop and reset motors to begin counting movement
        stopRobot();
        sleep(10);
        left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target stop and mode for running to a position
        left_front_motor.setTargetPosition(targetCount);
        right_front_motor.setTargetPosition(-targetCount);
        left_rear_motor.setTargetPosition(-targetCount);
        right_rear_motor.setTargetPosition(targetCount);

        left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //conitnue driving until all of the motors hits the distance
        while (left_front_motor.isBusy() || right_front_motor.isBusy() || left_rear_motor.isBusy()
                || right_rear_motor.isBusy()) {
            // Send calculated power to wheels
            left_front_motor.setPower(targetSpeed);
            right_front_motor.setPower(targetSpeed);
            left_rear_motor.setPower(targetSpeed);
            right_rear_motor.setPower(targetSpeed);
        }//end while

        stopRobot();
        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }  //end drive robot distance strafe right

    /***
     * Routine to drive a robot forward a certain distance and stop
     * @param distanceInCM
     * @param targetSpeed
     */
    public void driveRobotDistanceStrafeLeft(double distanceInCM, double targetSpeed) {
        int targetCount;

        //convert sentimeters to number cycles to drive
        //counts_per_rotation/circumference*distane
        targetCount = (int) Math.round(COUNTS_PER_GEAR_REV*1.1 / WHEEL_CIRCUMFERENCE * distanceInCM);

        //ensure full stop and reset motors to begin counting movement
        stopRobot();
        sleep(10);
        left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target stop and mode for running to a position
        left_front_motor.setTargetPosition(-targetCount);
        right_front_motor.setTargetPosition(targetCount);
        left_rear_motor.setTargetPosition(targetCount);
        right_rear_motor.setTargetPosition(-targetCount);

        left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //conitnue driving until all of the motors hits the distance
        while (left_front_motor.isBusy() || right_front_motor.isBusy()) {
            // Send calculated power to wheels
            left_front_motor.setPower(targetSpeed);
            right_front_motor.setPower(targetSpeed);
            left_rear_motor.setPower(targetSpeed);
            right_rear_motor.setPower(targetSpeed);
        }//end while

        stopRobot();
        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }  //end drive robot distance strafe right

    public void rotateRight90Degrees() {
        int targetCount;
        double targetSpeed = 0.5;
        double diameter = 56;//56.8;   //diameter in cms measured between left front and right rear or RF and LR

        //convert centimeters to number cycles to drive
        //to make a 90 degree turn, use diameter divide by four; so, diameter * pi / 4
        // counts_per_rotation/circumference*
        targetCount = (int) Math.round(COUNTS_PER_GEAR_REV / WHEEL_CIRCUMFERENCE * diameter * Math.PI / 4);

        //ensure full stop and reset motors to begin counting movement
        stopRobot();
        sleep(10);
        left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //revere motors
      //  reverseMotor(right_front_motor);
      //  reverseMotor(right_rear_motor);

        //set target stop and mode for running to a position
        left_front_motor.setTargetPosition(targetCount);
        right_front_motor.setTargetPosition(-targetCount);
        left_rear_motor.setTargetPosition(targetCount);
        right_rear_motor.setTargetPosition(-targetCount);

        left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //continue driving until all of the motors hits the distance
        while (left_front_motor.isBusy() || right_front_motor.isBusy() || left_rear_motor.isBusy()
                || right_rear_motor.isBusy()) {
            // Send calculated power to wheels
            left_front_motor.setPower(targetSpeed);
            right_front_motor.setPower(targetSpeed);
            left_rear_motor.setPower(targetSpeed);
            right_rear_motor.setPower(targetSpeed);
        }//end while

        stopRobot();

        //return to normal motors
       // reverseMotor(right_front_motor);
       // reverseMotor(right_rear_motor);

        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rotateRight90DegreesRobotTwo() {
        int targetCount;
        double targetSpeed = 0.5;
        double diameter = 39;   //diameter in cms measured between left front and right rear or RF and LR

        //convert centimeters to number cycles to drive
        //to make a 90 degree turn, use diameter divide by four; so, diameter * pi / 4
        // counts_per_rotation/circumference*
        targetCount = (int) Math.round(COUNTS_PER_GEAR_REV / WHEEL_CIRCUMFERENCE * diameter * Math.PI / 4);

        //ensure full stop and reset motors to begin counting movement
        stopRobot();
        sleep(10);
        left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //revere motors
        //  reverseMotor(right_front_motor);
        //  reverseMotor(right_rear_motor);

        //set target stop and mode for running to a position
        left_front_motor.setTargetPosition(targetCount);
        right_front_motor.setTargetPosition(-targetCount);
        left_rear_motor.setTargetPosition(targetCount);
        right_rear_motor.setTargetPosition(-targetCount);

        left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //continue driving until all of the motors hits the distance
        while (left_front_motor.isBusy() || right_front_motor.isBusy() || left_rear_motor.isBusy()
                || right_rear_motor.isBusy()) {
            // Send calculated power to wheels
            left_front_motor.setPower(targetSpeed);
            right_front_motor.setPower(targetSpeed);
            left_rear_motor.setPower(targetSpeed);
            right_rear_motor.setPower(targetSpeed);
        }//end while

        stopRobot();

        //return to normal motors
        // reverseMotor(right_front_motor);
        // reverseMotor(right_rear_motor);

        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rotateLeft90Degrees() {
        int targetCount;
        double targetSpeed = 0.5;
        double diameter = 56.8;//56.8;   //diameter in cms measured between left front and right rear or RF and LR

        //convert centimeters to number cycles to drive
        //to make a 90 degree turn, use diameter divide by four; so, diameter * pi / 4
        // counts_per_rotation/circumference*
        targetCount = (int) Math.round(COUNTS_PER_GEAR_REV  / WHEEL_CIRCUMFERENCE * diameter * Math.PI / 4);

        //ensure full stop and reset motors to begin counting movement
        stopRobot();
        sleep(10);
        left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //revere motors
        //reverseMotor(left_front_motor);
        //reverseMotor(left_rear_motor);

        //set target stop and mode for running to a position
        left_front_motor.setTargetPosition(-targetCount);
        right_front_motor.setTargetPosition(targetCount);
        left_rear_motor.setTargetPosition(-targetCount);
        right_rear_motor.setTargetPosition(targetCount);

        left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //conitnue driving until all of the motors hits the distance
        while (left_front_motor.isBusy() || right_front_motor.isBusy() || left_rear_motor.isBusy()
                || right_rear_motor.isBusy()) {
            // Send calculated power to wheels
            left_front_motor.setPower(targetSpeed);
            right_front_motor.setPower(targetSpeed);
            left_rear_motor.setPower(targetSpeed);
            right_rear_motor.setPower(targetSpeed);
        }//end while

        stopRobot();

        //return to normal motors
       // reverseMotor(left_front_motor);
        //reverseMotor(left_rear_motor);

        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rotateLeft90DegreesRobotTwo() {
        int targetCount;
        double targetSpeed = 0.5;
        double diameter = 39;   //diameter in cms measured between left front and right rear or RF and LR

        //convert centimeters to number cycles to drive
        //to make a 90 degree turn, use diameter divide by four; so, diameter * pi / 4
        // counts_per_rotation/circumference*
        targetCount = (int) Math.round(COUNTS_PER_GEAR_REV  / WHEEL_CIRCUMFERENCE * diameter * Math.PI / 4);

        //ensure full stop and reset motors to begin counting movement
        stopRobot();
        sleep(10);
        left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //revere motors
        //reverseMotor(left_front_motor);
        //reverseMotor(left_rear_motor);

        //set target stop and mode for running to a position
        left_front_motor.setTargetPosition(-targetCount);
        right_front_motor.setTargetPosition(targetCount);
        left_rear_motor.setTargetPosition(-targetCount);
        right_rear_motor.setTargetPosition(targetCount);

        left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //conitnue driving until all of the motors hits the distance
        while (left_front_motor.isBusy() || right_front_motor.isBusy() || left_rear_motor.isBusy()
                || right_rear_motor.isBusy()) {
            // Send calculated power to wheels
            left_front_motor.setPower(targetSpeed);
            right_front_motor.setPower(targetSpeed);
            left_rear_motor.setPower(targetSpeed);
            right_rear_motor.setPower(targetSpeed);
        }//end while

        stopRobot();

        //return to normal motors
        // reverseMotor(left_front_motor);
        //reverseMotor(left_rear_motor);

        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rotateRight45Degrees() {
        int targetCount;
        double targetSpeed = 0.5;
        double diameter = 56;//56.8;   //diameter in cms measured between left front and right rear or RF and LR

        //convert centimeters to number cycles to drive
        //to make a 90 degree turn, use diameter divide by four; so, diameter * pi / 8
        // counts_per_rotation/circumference*
        targetCount = (int) Math.round(COUNTS_PER_GEAR_REV / WHEEL_CIRCUMFERENCE * diameter * Math.PI / 8);

        //ensure full stop and reset motors to begin counting movement
        stopRobot();
        sleep(10);
        left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //revere motors
        //  reverseMotor(right_front_motor);
        //  reverseMotor(right_rear_motor);

        //set target stop and mode for running to a position
        left_front_motor.setTargetPosition(targetCount);
        right_front_motor.setTargetPosition(-targetCount);
        left_rear_motor.setTargetPosition(targetCount);
        right_rear_motor.setTargetPosition(-targetCount);

        left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //continue driving until all of the motors hits the distance
        while (left_front_motor.isBusy() || right_front_motor.isBusy() || left_rear_motor.isBusy()
                || right_rear_motor.isBusy()) {
            // Send calculated power to wheels
            left_front_motor.setPower(targetSpeed);
            right_front_motor.setPower(targetSpeed);
            left_rear_motor.setPower(targetSpeed);
            right_rear_motor.setPower(targetSpeed);
        }//end while

        stopRobot();

        //return to normal motors
        // reverseMotor(right_front_motor);
        // reverseMotor(right_rear_motor);

        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void rotateRight45DegreesRobotTwo() {
        int targetCount;
        double targetSpeed = 0.5;
        double diameter = 39;   //diameter in cms measured between left front and right rear or RF and LR

        //convert centimeters to number cycles to drive
        //to make a 90 degree turn, use diameter divide by four; so, diameter * pi / 8
        // counts_per_rotation/circumference*
        targetCount = (int) Math.round((COUNTS_PER_GEAR_REV / WHEEL_CIRCUMFERENCE * diameter * Math.PI / 8));
        //targetCount = (int) Math.round(targetCount*(5/7));
        //ensure full stop and reset motors to begin counting movement
        stopRobot();
        sleep(10);
        left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //revere motors
        //  reverseMotor(right_front_motor);
        //  reverseMotor(right_rear_motor);

        //set target stop and mode for running to a position
        left_front_motor.setTargetPosition(targetCount);
        right_front_motor.setTargetPosition(-targetCount);
        left_rear_motor.setTargetPosition(targetCount);
        right_rear_motor.setTargetPosition(-targetCount);

        left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //continue driving until all of the motors hits the distance
        while (left_front_motor.isBusy() || right_front_motor.isBusy() || left_rear_motor.isBusy()
                || right_rear_motor.isBusy()) {
            // Send calculated power to wheels
            left_front_motor.setPower(targetSpeed);
            right_front_motor.setPower(targetSpeed);
            left_rear_motor.setPower(targetSpeed);
            right_rear_motor.setPower(targetSpeed);
        }//end while

        stopRobot();

        //return to normal motors
        // reverseMotor(right_front_motor);
        // reverseMotor(right_rear_motor);

        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rotateLeft45Degrees() {
        int targetCount;
        double targetSpeed = 0.5;
        double diameter = 56.8;//56.8;   //diameter in cms measured between left front and right rear or RF and LR

        //convert centimeters to number cycles to drive
        //to make a 90 degree turn, use diameter divide by four; so, diameter * pi / 8
        // counts_per_rotation/circumference*
        targetCount = (int) Math.round(COUNTS_PER_GEAR_REV  / WHEEL_CIRCUMFERENCE * diameter * Math.PI / 8);

        //ensure full stop and reset motors to begin counting movement
        stopRobot();
        sleep(10);
        left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //revere motors
        //reverseMotor(left_front_motor);
        //reverseMotor(left_rear_motor);

        //set target stop and mode for running to a position
        left_front_motor.setTargetPosition(-targetCount);
        right_front_motor.setTargetPosition(targetCount);
        left_rear_motor.setTargetPosition(-targetCount);
        right_rear_motor.setTargetPosition(targetCount);

        left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //conitnue driving until all of the motors hits the distance
        while (left_front_motor.isBusy() || right_front_motor.isBusy() || left_rear_motor.isBusy()
                || right_rear_motor.isBusy()) {
            // Send calculated power to wheels
            left_front_motor.setPower(targetSpeed);
            right_front_motor.setPower(targetSpeed);
            left_rear_motor.setPower(targetSpeed);
            right_rear_motor.setPower(targetSpeed);
        }//end while

        stopRobot();

        //return to normal motors
        // reverseMotor(left_front_motor);
        //reverseMotor(left_rear_motor);

        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void rotateLeft45DegreesRobotTwo() {
        int targetCount;
        double targetSpeed = 0.5;
        double diameter = 39;   //diameter in cms measured between left front and right rear or RF and LR

        //convert centimeters to number cycles to drive
        //to make a 90 degree turn, use diameter divide by four; so, diameter * pi / 8
        // counts_per_rotation/circumference*
        targetCount = (int) Math.round((COUNTS_PER_GEAR_REV  / WHEEL_CIRCUMFERENCE * diameter * Math.PI / 8));
        //targetCount = (int) Math.round(targetCount*(5/7));
        //ensure full stop and reset motors to begin counting movement
        stopRobot();
        sleep(10);
        left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //revere motors
        //reverseMotor(left_front_motor);
        //reverseMotor(left_rear_motor);

        //set target stop and mode for running to a position
        left_front_motor.setTargetPosition(-targetCount);
        right_front_motor.setTargetPosition(targetCount);
        left_rear_motor.setTargetPosition(-targetCount);
        right_rear_motor.setTargetPosition(targetCount);

        left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //conitnue driving until all of the motors hits the distance
        while (left_front_motor.isBusy() || right_front_motor.isBusy() || left_rear_motor.isBusy()
                || right_rear_motor.isBusy()) {
            // Send calculated power to wheels
            left_front_motor.setPower(targetSpeed);
            right_front_motor.setPower(targetSpeed);
            left_rear_motor.setPower(targetSpeed);
            right_rear_motor.setPower(targetSpeed);
        }//end while

        stopRobot();

        //return to normal motors
        // reverseMotor(left_front_motor);
        //reverseMotor(left_rear_motor);

        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /****************
     * Use this to reverse the motor direction so encoding will work
     * @param thisMotor
     * @return reversed motor
     */
    DcMotor reverseMotor(DcMotor thisMotor) {
        if(thisMotor.getDirection()==DcMotor.Direction.FORWARD) thisMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        else thisMotor.setDirection(DcMotor.Direction.FORWARD);

        return thisMotor;
    }

    public void setMotorPowers(double lf, double lr, double rr, double rf) {

        left_front_motor.setPower(lf);
        left_rear_motor.setPower(lr);
        right_rear_motor.setPower(rr);
        right_front_motor.setPower(rf);
        //telemetry.addData("setmotorpowers", v);
    }

    //Set power to all motors
    public void setAllPower(double p){
        setMotorPowers(p,p,p,p);
    }

    public void tankDrive(double left_stick_x, double left_stick_y, double right_stick_x, double right_stick_y, double DRIVE_SPEED){
      /* This enables a tank drive-like arrangement where the left_stick controls the left
            wheels and the right_stick controls the right wheels uses it x/y values and hypotenuse
            to assign magnitude to the stick_y and stick_x values.  Avoids divide by 0 by checking
            hypotenuse
        */
        L_HYPOTENUSE = Math.sqrt(left_stick_y * left_stick_y +
                left_stick_x * left_stick_x);
        if (L_HYPOTENUSE == 0) {
            LEFT_FRONT_POWER = 0;
            LEFT_REAR_POWER = 0;
        } else {
            LEFT_FRONT_POWER = -left_stick_y *
                    Math.abs(left_stick_y / L_HYPOTENUSE);
            LEFT_FRONT_POWER += left_stick_x *
                    Math.abs(left_stick_x / L_HYPOTENUSE);
            LEFT_REAR_POWER = -left_stick_y *
                    Math.abs(left_stick_y / L_HYPOTENUSE);
            LEFT_REAR_POWER -= left_stick_x *
                    Math.abs(left_stick_x / L_HYPOTENUSE);
        }
        R_HYPOTENUSE = Math.sqrt(right_stick_y * right_stick_y +
                right_stick_x * right_stick_x);
        if (R_HYPOTENUSE == 0) {
            RIGHT_FRONT_POWER = 0;
            RIGHT_REAR_POWER = 0;
        } else {
            RIGHT_FRONT_POWER = -right_stick_y *
                    Math.abs(right_stick_y / R_HYPOTENUSE);
            RIGHT_FRONT_POWER += right_stick_x *
                    Math.abs(right_stick_x / R_HYPOTENUSE);
            RIGHT_REAR_POWER = -right_stick_y *
                    Math.abs(right_stick_y / R_HYPOTENUSE);
            RIGHT_REAR_POWER -= right_stick_x *
                    Math.abs(right_stick_x / R_HYPOTENUSE);
        }

        //Ensure Power is between -1 and 1, then factor down by DRIVE_SPEED
        LEFT_FRONT_POWER = Range.clip(LEFT_FRONT_POWER, -1.0, 1.0) * DRIVE_SPEED;
        LEFT_REAR_POWER = Range.clip(LEFT_REAR_POWER, -1.0, 1.0) * DRIVE_SPEED;
        RIGHT_FRONT_POWER = Range.clip(RIGHT_FRONT_POWER, -1.0, 1.0) * DRIVE_SPEED;
        RIGHT_REAR_POWER = Range.clip(RIGHT_REAR_POWER, -1.0, 1.0) * DRIVE_SPEED;

        // Send calculated power to wheels
        setMotorPowers(LEFT_FRONT_POWER,LEFT_REAR_POWER,RIGHT_REAR_POWER,RIGHT_FRONT_POWER);
    }

    public void arcadeDrive(double left_stick_x, double left_stick_y, double right_stick_x, double right_stick_y, double DRIVE_SPEED) {
        double y = -left_stick_y * DRIVE_SPEED; // Remember, this is reversed!
        double x = left_stick_x * 1.1 *DRIVE_SPEED; // Counteract imperfect strafing
        double rx = right_stick_x * DRIVE_SPEED;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        setMotorPowers(frontLeftPower,backLeftPower,backRightPower,frontRightPower);
    }
}   //end program
