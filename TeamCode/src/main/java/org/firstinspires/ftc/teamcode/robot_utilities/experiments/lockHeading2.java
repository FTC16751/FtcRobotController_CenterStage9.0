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

public class lockHeading2 {

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
    private double lockedHeading; // The heading angle to maintain when locked
    private boolean lockButtonState = false; // Tracks previous state of the lock button
    private double targetHeading = 0; // Desired heading for PID control
    private double lastLockButtonTime = 0; // Time of last lock button press
    private final double lockButtonDebounceTime = 0.2; // Debounce time for lock button (seconds)
    private double lastIMUHeading = 0;
    private final double maxTurnCorrection = 0.1; // Maximum allowed turn correction
    private static final double Kp = 0.9; // Proportional gain for heading correction

    public lockHeading2(HardwareMap hardwareMap, Telemetry telemetry) {
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

        // Initialize the IMU (gyroscope)
        initializeIMU(hardwareMap);
    }

    private void initializeIMU(HardwareMap hardwareMap) {
        // Retrieve and initialize the IMU.


// This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Now initialize the IMU with this mounting orientation


// Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    Orientation myRobotOrientation;

    public void drive(double leftStickX, double leftStickY, double rightStickX, double rightStickY, boolean lockHeadingButton) {
        // Check for lock button press
        /*if (lockHeadingButton && System.currentTimeMillis() - lastLockButtonTime > lockButtonDebounceTime * 1000) {
            isHeadingLocked = !isHeadingLocked;
            if (isHeadingLocked) {
                lockedHeading = getIMUHeading();
            }
            lastLockButtonTime = System.currentTimeMillis();
        }*/
        if (isHeadingLockButtonJustPressed(lockHeadingButton)) {
            isHeadingLocked = !isHeadingLocked;
            if (isHeadingLocked) {
                telemetry.addData("updating Locke Heading: ", lockedHeading);
                lockedHeading = getIMUHeading(); // Store initial IMU reading
            }
        }
        if (isHeadingLocked) {
            // Update heading only when the lock is set initially, not during continuous lock
            targetHeading = lockedHeading;
        }
        if (lockHeadingButton && !isHeadingLockButtonJustPressed(lockHeadingButton)) {
            // Ensure the heading remains locked until the button is released
            isHeadingLocked = true;
            } else if (!lockHeadingButton) {
            // Release the heading lock when the button is released
            isHeadingLocked = false;
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

        // Update target heading and apply PID correction if heading lock is active
        if (isHeadingLocked) {
            // Calculate heading error
            //double headingError = lockedHeading - getIMUHeading();

            // Calculate heading error using the shortest angular distance
            double imuHeading = getIMUHeading();
            double headingError = getShortestAngularDistance(imuHeading, lockedHeading);

            // Update PID controller
            double turnCorrection = pidController.update(headingError);

            // Limit turn correction magnitude
            //turnCorrection = Math.min(maxTurnCorrection, Math.max(-maxTurnCorrection, turnCorrection));

            // Apply heading correction to turn power
            turn += turnCorrection;

            // Update desired motor powers with PID correction and limit to 1
            targetFLPower = Math.min(1.0, targetFLPower + turn);
            targetFRPower = Math.min(1.0, targetFRPower - turn);
            targetRLPower = Math.min(1.0, targetRLPower + turn);
            targetRRPower = Math.min(1.0, targetRRPower - turn);

            // Output telemetry data for debugging
            telemetry.addData("is heading locked: ", isHeadingLocked);
            telemetry.addData("targetHeading: ", targetHeading);
            telemetry.addData("lockedHeading: ", lockedHeading);
            telemetry.addData("imu heading:", getIMUHeading());
            telemetry.addData("headingError: ", headingError);
            telemetry.addData("turnCorrection: ", turnCorrection);
            telemetry.addData("turn: ", turn);
            telemetry.addData("Desired Front Left Power", Math.min(1.0, targetFLPower));
            telemetry.addData("Read Front Left Power", frontLeft.getPower());
            telemetry.addData("Desired Front Right Power", Math.min(1.0, targetFRPower));
            telemetry.addData("Read Front Right Power", frontRight.getPower());
            telemetry.addData("Desired Rear Left Power", Math.min(1.0, targetRLPower));
            telemetry.addData("Read Rear Left Power", rearLeft.getPower());
            telemetry.addData("Desired Rear Right Power", Math.min(1.0, targetRRPower));
            telemetry.addData("Read Rear Right Power", rearRight.getPower());
        }

        // Apply motor powers
        frontLeft.setPower(targetFLPower);
        frontRight.setPower(targetFRPower);
        rearLeft.setPower(targetRLPower);
        rearRight.setPower(targetRRPower);

        // Update last joystick inputs and time
        lastForwardReverse = forwardReverse;
        lastStrafe = strafe;
        lastTurn = turn;
        lastUpdateTime = System.currentTimeMillis() / 1000.0;

        // Update telemetry or perform other actions if needed
        // ...
    }
    public void updateTelemetry() {
        telemetry.addData("is heading locked: ", isHeadingLocked);
        telemetry.addData("targetHeading: ", targetHeading);
        telemetry.addData("lockedHeading: ", lockedHeading);
        // ... (other telemetry data)
    }
    private boolean previousLockButtonState = false; // Track previous state of the lock button

    private boolean isHeadingLockButtonJustPressed(boolean lockHeadingButton) {
        telemetry.addData("isHeadingLockButtonJustPressed: ", lockHeadingButton);
        telemetry.addData("isHeadingLockButtonJustPressed previousLockButtonState: ", previousLockButtonState);
        boolean isPressed = lockHeadingButton && !previousLockButtonState;
        previousLockButtonState = lockHeadingButton; // Update previous state
        return isPressed;
    }
    private PIDController pidController = new PIDController(0.010, 0.00, 0.0); // PID controller parameters (tuning required)
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
            double errorIntegral = errorSum + error; // Integrate error over time
            double errorDerivative = (error - previousError) / (System.currentTimeMillis() / 1000.0 - lastUpdateTime); // Calculate derivative of error
            double output = Kp * error + Ki * errorIntegral + Kd * errorDerivative; // Calculate PID correction
telemetry.addData("error: ", error);
telemetry.addData("kp*error: ", Kp*error);
            previousError = error; // Update previous error for next iteration
            errorSum = errorIntegral; // Update integral sum

            return output;
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
    private double lowPassFilter(double input, double previousValue, double alpha) {
        return alpha * input + (1 - alpha) * previousValue;
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
