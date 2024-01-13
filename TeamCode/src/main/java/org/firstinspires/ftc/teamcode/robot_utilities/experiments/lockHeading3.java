package org.firstinspires.ftc.teamcode.robot_utilities.experiments;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class lockHeading3 {
    private final DcMotorEx[] motors;
    private double rampRate = 0.9;
    private final double[] lastMotorPowers = new double[4];
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
    public lockHeading3(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        motors = new DcMotorEx[] {
                hardwareMap.get(DcMotorEx.class, "Front_Left"),
                hardwareMap.get(DcMotorEx.class, "Front_Right"),
                hardwareMap.get(DcMotorEx.class, "Rear_Left"),
                hardwareMap.get(DcMotorEx.class, "Rear_Right")
        };

        // Reverse left motors for driving
        motors[0].setDirection(DcMotorEx.Direction.REVERSE);
        motors[1].setDirection(DcMotorEx.Direction.FORWARD);
        motors[2].setDirection(DcMotorEx.Direction.REVERSE);
        motors[3].setDirection(DcMotorEx.Direction.FORWARD);

        // Set brake condition to full stop
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

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

        // Calculate heading error
        double headingError = getShortestAngularDistance(getIMUHeading(), desiredHeading);

        // Apply PID correction
        double turnCorrection = pidController.update(headingError);

        // Limit turn correction magnitude
        turnCorrection = Math.min(maxTurnCorrection, Math.max(-maxTurnCorrection, turnCorrection));

        // Calculate desired motor powers
        double[] targetMotorPowers = calculateMotorPowers(leftStickX, leftStickY, turnCorrection);

        // Apply ramped motor power
        setMotorPowers(targetMotorPowers);

        // Update telemetry
        updateTelemetry(headingError, turnCorrection, targetMotorPowers);
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
    }
    private double normalizeAngle(double angle) {
        if (angle < 0) {
            angle += 360;
        } else if (angle >= 360) {
            angle -= 360;
        }
        return angle;
    }

    private double[] calculateMotorPowers(double leftStickX, double leftStickY, double turnCorrection) {
        // Define maximum forward/backward and strafe speeds
        final double MAX_FORWARD_SPEED = 1.0;
        final double MAX_STRAFE_SPEED = 0.5;

        // Initialize target motor powers
        double[] targetMotorPowers = new double[4];

        // Calculate scaled forward/backward power
        double forwardBackwardPower = limitRange(leftStickY * MAX_FORWARD_SPEED, -MAX_FORWARD_SPEED, MAX_FORWARD_SPEED);

        // Calculate scaled strafe power
        double strafePower = limitRange(leftStickX * MAX_STRAFE_SPEED, -MAX_STRAFE_SPEED, MAX_STRAFE_SPEED);

        // Apply heading correction to turn power
        double turnPower = turnCorrection;

        // Calculate individual motor powers based on combined inputs, considering robot orientation
        double flPower = forwardBackwardPower + strafePower + turnPower; // Front Left
        double frPower = forwardBackwardPower - strafePower - turnPower; // Front Right
        double rlPower = forwardBackwardPower - strafePower + turnPower; // Rear Left
        double rrPower = forwardBackwardPower + strafePower - turnPower; // Rear Right

        // Apply a slight forward bias for smoother movement
        flPower += 0.1;
        frPower += 0.1;
        rlPower += 0.1;
        rrPower += 0.1;

        // Normalize motor powers if necessary (to avoid exceeding motor limits)
        double maxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.max(Math.abs(rlPower), Math.abs(rrPower)));
        if (maxPower > 1.0) {
            flPower /= maxPower;
            frPower /= maxPower;
            rlPower /= maxPower;
            rrPower /= maxPower;
        }

        // Apply ramped power to motors
        for (int i = 0; i < 4; i++) {
            targetMotorPowers[i] = rampPower(lastMotorPowers[i], targetMotorPowers[i], System.currentTimeMillis() / 1000.0);
            lastMotorPowers[i] = targetMotorPowers[i];
        }

        return targetMotorPowers;
    }

    private double limitRange(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }
    private double rampPower(double currentPower, double targetPower, double currentTime) {
        telemetry.addData("ramp up ramp up: ", "curr: " + currentPower  + "target: " + targetPower);
        double powerDifference = targetPower - currentPower;
        double rampedPower = currentPower + (powerDifference * rampRate * (currentTime - lastUpdateTime));
        return Math.min(1, Math.max(-1, rampedPower)); // Ensure power is within motor limits (-1 to 1)
    }
    private void setMotorPowers(double[] targetMotorPowers) {
        // Set motor powers with ramped values
        telemetry.addData("YOOOO IM SETTING POWER, ABOUT TO LOOP","before loop");
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(rampPower(lastMotorPowers[i], targetMotorPowers[i], System.currentTimeMillis() / 1000.0));
            telemetry.addData("YOOOO IM SETTING POWER, LOOP DE LOOP: ", motors[i] + " " + targetMotorPowers[i]);
            lastMotorPowers[i] = targetMotorPowers[i];
        }
    }

    private void updateTelemetry(double headingError, double turnCorrection, double[] targetMotorPowers) {
        // Update general telemetry data
        telemetry.addData("is heading locked:", isHeadingLocked);
        telemetry.addData("desired heading:", desiredHeading);
        telemetry.addData("locked heading:", lockedHeading);
        telemetry.addData("IMU heading:", getIMUHeading());
        telemetry.addData("heading error:", headingError);

        // Update motor specific telemetry data
        for (int i = 0; i < motors.length; i++) {
            telemetry.addData("Motor " + (i + 1) + " Target Power:", targetMotorPowers[i]);
            telemetry.addData("Motor " + (i + 1) + " Actual Power:", motors[i].getPower());
        }

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
