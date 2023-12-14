package org.firstinspires.ftc.teamcode.robot.utilities;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class SmoothDriveMovement2 {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx rearLeft;
    private DcMotorEx rearRight;
    private double rampRate = 0.9;
    private double lastForwardReverse = 0;
    private double lastStrafe = 0;
    private double lastTurn = 0;
    private double lastUpdateTime = 0; // Declare last update time
    private Telemetry telemetry;
    private IMU imu;
    private boolean isHeadingLocked = false;
    private double lockedHeading = 0; // The heading angle to maintain when locked
    private boolean lockButtonState = false; // Tracks previous state of the lock button
    private double targetHeading = 0; // Desired heading for PID control
    private double lastLockButtonTime = 0; // Time of last lock button press
    private final double lockButtonDebounceTime = 0.2; // Debounce time for lock button (seconds)
    private double lastIMUHeading = 0;
    private final double maxTurnCorrection = 0.5; // Maximum allowed turn correction
    private static final double Kp = 0.9; // Proportional gain for heading correction


    public SmoothDriveMovement2(HardwareMap hardwareMap, Telemetry telemetry ) {
        this.telemetry = telemetry;
        frontLeft = hardwareMap.get(DcMotorEx.class, "Front_Left");
        frontRight = hardwareMap.get(DcMotorEx.class, "Front_Right");
        rearLeft = hardwareMap.get(DcMotorEx.class, "Rear_Left");
        rearRight = hardwareMap.get(DcMotorEx.class, "Rear_Right");

        // Reverse left motors for driving (so it goes counterclockwise to drive forward)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        //Set brake condition to full stop (allso can give DcMotor.ZeroPowerBehavior.FLOAT)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //initialize the imu (gyroscope)
        initializeIMU(hardwareMap);
    }
    private void initializeIMU(HardwareMap hardwareMap) {
        // Retrieve and initialize the IMU.
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
    Orientation myRobotOrientation;

    public void drive(double leftStickX, double leftStickY, double rightStickX, double rightStickY) {
        double forwardReverse = leftStickY;
        double strafe = leftStickX;
        double turn = rightStickX;

        double currentTime = System.currentTimeMillis() / 1000.0; // Get time in seconds
/*
        if (Math.abs(leftStickX) < 0.05 && Math.abs(leftStickY) < 0.05 && Math.abs(rightStickX) < 0.05 && Math.abs(rightStickY) < 0.05) {
            // If no joystick input, gradually reduce power to 0
            frontLeft.setPower(rampPower(frontLeft.getPower(), 0, currentTime));
            frontRight.setPower(rampPower(frontRight.getPower(), 0, currentTime));
            rearLeft.setPower(rampPower(rearLeft.getPower(), 0, currentTime));
            rearRight.setPower(rampPower(rearRight.getPower(), 0, currentTime));
        } else {
            // Calculate target powers
            double targetFLPower = forwardReverse + strafe + turn;
            double targetFRPower = forwardReverse - strafe - turn;
            double targetRLPower = forwardReverse - strafe + turn;
            double targetRRPower = forwardReverse + strafe - turn;

            // Ramp powers from current to target gradually
            frontLeft.setPower(rampPower(frontLeft.getPower(), targetFLPower, currentTime));
            frontRight.setPower(rampPower(frontRight.getPower(), targetFRPower, currentTime));
            rearLeft.setPower(rampPower(rearLeft.getPower(), targetRLPower, currentTime));
            rearRight.setPower(rampPower(rearRight.getPower(), targetRRPower, currentTime));
        }

 */
        if (Math.abs(leftStickX) < 0.05 && Math.abs(leftStickY) < 0.05 && Math.abs(rightStickX) < 0.05 && Math.abs(rightStickY) < 0.05) {
            // If no joystick input, gradually reduce power to 0
            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);
        } else {
            // Calculate target powers
            double targetFLPower = forwardReverse + strafe + turn;
            double targetFRPower = forwardReverse - strafe - turn;
            double targetRLPower = forwardReverse - strafe + turn;
            double targetRRPower = forwardReverse + strafe - turn;

            // Ramp powers from current to target gradually
            frontLeft.setPower(rampPower(frontLeft.getPower(), targetFLPower, currentTime));
            frontRight.setPower(rampPower(frontRight.getPower(), targetFRPower, currentTime));
            rearLeft.setPower(rampPower(rearLeft.getPower(), targetRLPower, currentTime));
            rearRight.setPower(rampPower(rearRight.getPower(), targetRRPower, currentTime));
        }
        // Update last joystick inputs
        lastForwardReverse = forwardReverse;
        lastStrafe = strafe;
        lastTurn = turn;
        lastUpdateTime = currentTime;

        // Output telemetry data if needed
        //telemetry.addData("Front Left Power", frontLeft.getPower());
       // telemetry.addData("Front Right Power", frontRight.getPower());
       // telemetry.addData("Rear Left Power", rearLeft.getPower());
       // telemetry.addData("Rear Right Power", rearRight.getPower());
    }

    public void driveStraight(double leftStickX, double leftStickY, double rightStickX, double rightStickY, boolean lockHeadingButton) {
        // Check if the lock button state changed from released to pressed
        // Check for lock button press
        if (lockHeadingButton && System.currentTimeMillis() - lastLockButtonTime > lockButtonDebounceTime * 1000) {
            isHeadingLocked = !isHeadingLocked;
            if (isHeadingLocked) {
                lockedHeading = getIMUHeading();
            }
            lastLockButtonTime = System.currentTimeMillis();
        }
        // Update the button state
        lockButtonState = lockHeadingButton;

        // Get joystick inputs
        double forwardReverse = leftStickY;
        double strafe = leftStickX;
        double turn = rightStickX;

        // Regular motor power calculations (without heading lock)
        double targetFLPower = forwardReverse + strafe + turn;
        double targetFRPower = forwardReverse - strafe - turn;
        double targetRLPower = forwardReverse - strafe + turn;
        double targetRRPower = forwardReverse + strafe - turn;

        if (isHeadingLockButtonJustPressed(lockHeadingButton)) {
            isHeadingLocked = !isHeadingLocked;
            if (isHeadingLocked) {
                // Calculate heading error
                double headingError = lockedHeading - getIMUHeading();

                // Update PID controller
                double turnCorrection = pidController.update(headingError);
                // Scale turn correction to limit its magnitude
                turnCorrection = Math.min(maxTurnCorrection, Math.max(-maxTurnCorrection, turnCorrection));
                //double headingDifference = lockedHeading - getIMUHeading();
                // Apply heading correction only when the heading is locked
                // Adjust the robot's turning to maintain the locked heading
                //turn = headingDifference * Kp; // Apply proportional correction

                // Apply motor powers with PID correction and limit to 1
                frontLeft.setPower(Math.min(1.0, targetFLPower + .5));
                frontRight.setPower(Math.min(1.0, targetFRPower - .5));
                rearLeft.setPower(Math.min(1.0, targetRLPower + .5));
                rearRight.setPower(Math.min(1.0, targetRRPower - .5));
            /*
            frontLeft.setPower(Math.min(1.0, targetFLPower + turnCorrection));
            frontRight.setPower(Math.min(1.0, targetFRPower - turnCorrection));
            rearLeft.setPower(Math.min(1.0, targetRLPower + turnCorrection));
            rearRight.setPower(Math.min(1.0, targetRRPower - turnCorrection));

             */
                telemetry.addData("targetHeading: ", targetHeading);
                telemetry.addData("targetHeading: ", lockedHeading);

                telemetry.addData("imu heading:", getIMUHeading());
                telemetry.addData("headingError: ", headingError);
                telemetry.addData("turnCorrection: ", turnCorrection);
                telemetry.addData("turn: ", turn);
                telemetry.addData("Desired Front Left Power", Math.min(1.0, targetFLPower + .5));
                telemetry.addData("Read Front Left Power", frontLeft.getPower());

                telemetry.addData("Desired Front Right Power", Math.min(1.0, targetFRPower - .5));
                telemetry.addData("Read Front Right Power", frontRight.getPower());

                telemetry.addData("Desired Rear Left Power", Math.min(1.0, targetRLPower + .5));
                telemetry.addData("Read Rear Left Power", rearLeft.getPower());

                telemetry.addData("Desired Rear Right Power", Math.min(1.0, targetRRPower - .5));
                telemetry.addData("Red Rear Right Power", rearRight.getPower());
            }
        }previousLockButtonState = lockHeadingButton;

        // Apply motor powers (with or without heading correction)
        frontLeft.setPower(targetFLPower + turn);
        frontRight.setPower(targetFRPower - turn);
        rearLeft.setPower(targetRLPower + turn);
        rearRight.setPower(targetRRPower - turn);

        // Update telemetry or perform other actions if needed
        // ...
    }
    boolean previousLockButtonState = false; // Track previous button state
    boolean isHeadingLockButtonJustPressed(boolean lockHeadingButton) {
        return lockHeadingButton && !previousLockButtonState;
    }
    private PIDController pidController = new PIDController(0.5, 0.01, 0.2); // PID controller parameters (tuning required)
    private class PIDController {
        private double Kp, Ki, Kd;
        private double errorSum = 0;
        private double previousError = 0;

        public PIDController(double Kp, double Ki, double Kd) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
        }

        public double update(double error) {
            double errorDelta = error - previousError;
            previousError = error;
            errorSum += error;

            return Kp * error + Ki * errorSum + Kd * errorDelta;
        }
    }
    private double lowPassFilter(double input, double previousValue, double alpha) {
        return alpha * input + (1 - alpha) * previousValue;
    }
    private double getIMUHeading() {
        // Filter IMU heading data
        return lowPassFilter(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle, lastIMUHeading, 0.1);
    }
    /*
    private double getIMUHeading() {

        // Your implementation to retrieve the IMU heading
        // Return the heading in degrees or radians based on your needs
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ,AngleUnit.DEGREES).firstAngle;
    }

     */
    private double rampPower(double currentPower, double targetPower, double currentTime) {
        double powerDifference = targetPower - currentPower;
        double rampedPower = currentPower + (powerDifference * rampRate * (currentTime - lastUpdateTime));
        return Math.min(1, Math.max(-1, rampedPower)); // Ensure power is within motor limits (-1 to 1)
    }
    public double getRampRate() {
        return rampRate;
    }
    public void setRampRate(double rampRateIncrement) {
        rampRate = rampRate + rampRateIncrement;
    }
    public double getMotorPower(DcMotorEx motor) {
        return motor.getPower();
    }
}
