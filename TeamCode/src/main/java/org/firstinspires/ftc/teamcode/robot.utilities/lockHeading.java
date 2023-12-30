package org.firstinspires.ftc.teamcode.robot.utilities;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class lockHeading {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx rearLeft;
    private DcMotorEx rearRight;
    private double rampRate = 0.9;
    private final double[] lastMotorPowers = new double[4];
    private double lastMotorPower_fl = 0;
    private double lastMotorPower_fr = 0;
    private double lastMotorPower_rl = 0;
    private double lastMotorPower_rr = 0;
    private final double lastUpdateTime = 0;
    private final Telemetry telemetry;
    private IMU imu;
    private boolean isHeadingLocked = false;
    private double lockedHeading;
    private boolean lockButtonState = false;
    private final double targetHeading = 0; // Desired heading for PID control
    private double desiredHeading;
    private double lastLockButtonTime = 0;
    private final double lockButtonDebounceTime = 0.2; // Debounce time for lock button (seconds)
    private final double maxTurnCorrection = 0.1; // Maximum allowed turn correction
    private final PIDController pidController = new PIDController(0.01, 0.0, 0.0); // PID controller parameters (tuning required)
    private double targetMotorPower_fl = 0;
    private double targetMotorPower_fr = 0;
    private double targetMotorPower_rl = 0;
    private double targetMotorPower_rr = 0;


    public lockHeading(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
         frontLeft   = hardwareMap.get(DcMotorEx.class, "Front_Left");
         frontRight  = hardwareMap.get(DcMotorEx.class, "Front_Right");
         rearLeft    = hardwareMap.get(DcMotorEx.class, "Rear_Left");
         rearRight   = hardwareMap.get(DcMotorEx.class, "Rear_Right");

        // Reverse left motors for driving
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        rearLeft.setDirection(DcMotorEx.Direction.REVERSE);
        rearRight.setDirection(DcMotorEx.Direction.FORWARD);

        // Set brake condition to full stop (also can give DcMotor.ZeroPowerBehavior.FLOAT)
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Initialize the IMU
        initializeIMU(hardwareMap);
    }
    private void initializeIMU(HardwareMap hardwareMap) {
        // Retrieve and initialize the IMU.
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
    Orientation myRobotOrientation;
    public void drive(double leftStickX, double leftStickY, double rightStickX, double rightStickY, boolean lockHeadingButton) {
        // Update heading lock state
        updateHeadingLockState(lockHeadingButton);

        // Update desired heading
        updateDesiredHeading(lockHeadingButton);
        double headingError=0;
        double turnCorrection=rightStickX;
        if (isHeadingLocked){
            // Calculate heading error
            headingError = getShortestAngularDistance(getIMUHeading(), desiredHeading);
            telemetry.addData("drive, heading error after calling shortest distance  ", headingError);

            // Apply PID correction
            turnCorrection = pidController.update(headingError);
            telemetry.addData("drive, turn correction after pid  ", turnCorrection);

            // Limit turn correction magnitude
            turnCorrection = Math.min(maxTurnCorrection, Math.max(-maxTurnCorrection, turnCorrection));
            telemetry.addData("drive, turn correction after min/max  ", turnCorrection);
        }

        // Calculate desired motor powers
        // {
            // Define maximum forward/backward and strafe speeds
            final double MAX_FORWARD_SPEED = 0.5;
            final double MAX_STRAFE_SPEED = 0.5;

            // Initialize target motor powers
            targetMotorPower_fl = 0;
            targetMotorPower_fr = 0;
            targetMotorPower_rl = 0;
            targetMotorPower_rr = 0;

            // Calculate scaled forward/backward power
            double forwardBackwardPower = limitRange(leftStickY * MAX_FORWARD_SPEED, -MAX_FORWARD_SPEED, MAX_FORWARD_SPEED);

            // Calculate scaled strafe power
            double strafePower = limitRange(leftStickX * MAX_STRAFE_SPEED, -MAX_STRAFE_SPEED, MAX_STRAFE_SPEED);

            // Apply heading correction to turn power
            double turnPower = turnCorrection;

            // Calculate individual motor powers based on combined inputs, considering robot orientation
            targetMotorPower_fl = forwardBackwardPower + strafePower + turnPower; // Front Left
            targetMotorPower_fr = forwardBackwardPower - strafePower - turnPower; // Front Right
            targetMotorPower_rl = forwardBackwardPower - strafePower + turnPower; // Rear Left
            targetMotorPower_rr = forwardBackwardPower + strafePower - turnPower; // Rear Right

            // Normalize motor powers if necessary (to avoid exceeding motor limits)
            double maxPower = Math.max(Math.max(Math.abs(targetMotorPower_fl), Math.abs(targetMotorPower_fr)), Math.max(Math.abs(targetMotorPower_rl), Math.abs(targetMotorPower_rr)));
            if (maxPower > 1.0) {
                targetMotorPower_fl /= maxPower;
                targetMotorPower_fr /= maxPower;
                targetMotorPower_rl /= maxPower;
                targetMotorPower_rr /= maxPower;
            }
        //}

        // Apply motor power
        setMotorPowers(
                targetMotorPower_fl,
                targetMotorPower_fr,
                targetMotorPower_rl,
                targetMotorPower_rr
        );

        // Update telemetry
        updateTelemetry(headingError, turnCorrection);
    }
    private void updateHeadingLockState(boolean lockHeadingButton) {
        if (isHeadingLockButtonJustPressed(lockHeadingButton)) {
            isHeadingLocked = !isHeadingLocked;
            if (isHeadingLocked) {
                lockedHeading = getIMUHeading();
            }
        }
        lockButtonState = lockHeadingButton;
    }

    private boolean isHeadingLockButtonJustPressed(boolean lockHeadingButton) {
        // Check if lock button state changed from false to true
        if (!lockButtonState && lockHeadingButton) {
            // Check debounce time
            if (System.currentTimeMillis() - lastLockButtonTime > lockButtonDebounceTime * 1000) {
                lastLockButtonTime = System.currentTimeMillis();
                return true;
            }
        } else {
            lastLockButtonTime = 0;
        }
        return false;
    }
    private void updateDesiredHeading(boolean lockHeadingButton) {
        if (isHeadingLocked) {
            desiredHeading = lockedHeading;
        } else if (!lockHeadingButton && desiredHeading != 0) {
            desiredHeading = 0;
        }
    }
    private double getIMUHeading() {
        // Get the orientation from the IMU
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // Extract the yaw value for heading
        double yawAngle = myRobotOrientation.firstAngle;

        // Adjust yaw angle to be between 0 and 360 degrees
        if (yawAngle < 0) {
            yawAngle += 360;
        }

        return yawAngle;
    }
    private double getShortestAngularDistance(double from, double to) {
      /*prior version
        // Convert angles to range between 0 and 360 degrees

        from = normalizeAngle(from);
        to = normalizeAngle(to);

        // Calculate the absolute difference between the angles
        double absoluteDifference = Math.abs(to - from);

        // Check if the difference is greater than 180 degrees
        if (absoluteDifference > 180) {
            // Calculate the shorter distance via the other direction
            return Math.min(360 - absoluteDifference, absoluteDifference);
        } else {
            // Return the absolute difference
            return absoluteDifference;
        }

       */
        //new refactored version
        // Convert angles to range between 0 and 360 degrees
        from = normalizeAngle(from);
        to = normalizeAngle(to);

        // Calculate the absolute difference between the angles without normalizing
        double absoluteDifference = Math.abs(to - from);

        // Check if the difference is greater than 180 degrees (wraps around the circle)
        telemetry.addData("getShortestAngularDistance: absoluteDifference1  ", absoluteDifference);
        if (absoluteDifference > 180) {
            // Calculate the shorter distance via the other direction on the circle
            double shorterDistanceViaOppositeDirection = 360 - absoluteDifference;

            // Compare the two distances and return the shorter one
            telemetry.addData("getShortestAngularDistance: absoluteDifference2  ", absoluteDifference);
            telemetry.addData("getShortestAngularDistance: shorterDistanceViaOppositeDirection  ", shorterDistanceViaOppositeDirection);
            return -Math.min(absoluteDifference, shorterDistanceViaOppositeDirection);
        } else {
            // If the difference is less than 180 degrees, the absolute difference is the shortest distance
            return absoluteDifference;
        }
    }
    private double normalizeAngle(double angle) {
        if (angle < 0) {
            angle += 360;
        } else if (angle >= 360) {
            angle -= 360;
        }
        return angle;
    }


    private double limitRange(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    private void setMotorPowers(double fl_target, double fr_target , double rl_target,double rr_target) {
        // Set motor powers with ramped values
        telemetry.addData("YOOOO IM SETTING POWER, ","biz");
        this.frontLeft.setPower(fl_target);
        this.frontRight.setPower(fr_target);
        this.rearLeft.setPower(rl_target);
        this.rearRight.setPower(rr_target);
    }

    private void updateTelemetry(double headingError, double turnCorrection) {
        // Update general telemetry data
        telemetry.addData("is heading locked:", isHeadingLocked);
        telemetry.addData("desired heading:", desiredHeading);
        telemetry.addData("locked heading:", lockedHeading);
        telemetry.addData("IMU heading:", getIMUHeading());
        telemetry.addData("heading error:", headingError);


        // Update motor specific telemetry data
        telemetry.addData("Motor Front Left" + " Target Power:", targetMotorPower_fl);
        telemetry.addData("Motor Front Left" + " Actual Power:", frontLeft.getPower());

        telemetry.addData("Motor Front Right" + " Target Power:", targetMotorPower_fr);
        telemetry.addData("Motor Front Right" + " Actual Power:", rearRight.getPower());

        telemetry.addData("Motor Rear Left" + " Target Power:", targetMotorPower_rl);
        telemetry.addData("Motor Rear Left" + " Actual Power:", rearLeft.getPower());

        telemetry.addData("Motor Rear Right" + " Target Power:", targetMotorPower_rr);
        telemetry.addData("Motor Rear Right" + " Actual Power:", rearRight.getPower());

        // Update additional telemetry data (optional)
        telemetry.addData("turn correction:", turnCorrection);
        telemetry.addData("ramp rate:", rampRate);
        telemetry.addData("last update time:", lastUpdateTime);
    }

    private class PIDController {
        private final double Kp;
        private final double Ki;
        private final double Kd;
        private double errorSum = 0;
        private double previousError = 0;
        public PIDController(double Kp, double Ki, double Kd) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
        }
        public double update(double error) {
            double errorIntegral = errorSum + error; // Integrate error over time
            double errorDerivative = (error - previousError) / (System.currentTimeMillis() / 1000.0 - lastUpdateTime); // Calculate derivative of error
            double output = Kp * error + Ki * errorIntegral + Kd * errorDerivative; // Calculate PID correction
            previousError = error; // Update previous error for next iteration
            errorSum = errorIntegral; // Update integral sum
            return output;
        }
    }
    public void setRampRate(double rampRateIncrement) {
        rampRate = rampRate + rampRateIncrement;
    }

}
