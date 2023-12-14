package org.firstinspires.ftc.teamcode.robot.utilities;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class NewDriveUtil2024 {

    // Define constants
    private static final double ENCODER_COUNTS_PER_INCH = 45.33;
    //Diameter = 96mm = 3.77953 inches (approximately)
    // Circumference = π * Diameter Circumference = π * 3.77953 inches ≈ 11.87 inches
    //Counts per inch = Encoder Resolution / Circumference
    //Counts per inch = 537.7 / 11.87 ≈ 45.33
    private final double ENCODER_COUNTS_PER_DEGREE; //SET IN CONSTRUCTOR //537.7 / 360; //≈ 1.4931;
    private final double WHEEL_DIAMETER = 3.77953;
    private final int ENCODER_RESOLUTION = 537;
    private final double GEAR_RATIO = 1;
    private static final double MAX_MOTOR_SPEED = 312;
    private static final double MOTOR_ACCELERATION = 1;

    // PID constants
    private static final double Kp = 0.5;
    private static final double Ki = 0.01;
    private static final double Kd = 0.1;

    // Ramp parameters
    private static final double RAMP_UP_TIME = 0.2; // Ramp-up time in seconds
    private static final double RAMP_DOWN_TIME = 0.2; // Ramp-down time in seconds
    private static final double K_ARC = 0.1; // Adjust the value according to your requirement


    // Class members
    public final DcMotor frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;
    private final ElapsedTime runtime;

    private final double ERROR_THRESHOLD = 10; // Define appropriate threshold based on encoder resolution and desired accuracy
    double errorLeft, errorRight;
    double derivativeLeft, derivativeRight;
    private double rampRate;

    public enum MotorType {FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT}

    // Assuming you have an IMU object initialized
    private IMU imu;

    //drive utility constructor
    public NewDriveUtil2024(HardwareMap hardwareMap) {

        // Initialize motor objects
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front_Left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front_Right");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "Rear_Left");
        rearRightMotor = hardwareMap.get(DcMotor.class, "Rear_Right");

        // Set motor directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        runtime = new ElapsedTime();
        // Define ENCODER_COUNTS_PER_DEGREE based on your robot configuration
        // This depends on factors like wheel diameter, encoder resolution, and gear ratios
        ENCODER_COUNTS_PER_DEGREE = calculateEncoderCountsPerDegree(WHEEL_DIAMETER, ENCODER_RESOLUTION, GEAR_RATIO);

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

    public double getimuAxisOrientation(Axis axis) {
        myRobotOrientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES);

        // Then read or display the desired values (Java type float):
        float X_axis = myRobotOrientation.firstAngle;
        float Y_axis = myRobotOrientation.secondAngle;
        float Z_axis = myRobotOrientation.thirdAngle;

        switch (axis) {
            case X:
                return X_axis;
            case Y:
                return Y_axis;
            case Z:
                return Z_axis;
            default:
                return X_axis;
        }
    }
    enum Axis {
        X, Y, Z
    }

    private double getIMUHeading() {
        // Your implementation to retrieve the IMU heading
        // Return the heading in degrees or radians based on your needs
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ,AngleUnit.DEGREES).firstAngle;
    }
    private double getHeading() {
        // Convert average encoder position to degrees
        double angle = getAverageEncoderAngle();

        // Retrieve IMU heading
        double imuHeading = getIMUHeading();

        // Apply gyro correction from the IMU if available
        angle += imuHeading;

        // Normalize angle to 0-360 degrees
        angle %= 360;

        return angle;
    }
    public double getHeading2() {
        // Ensure robot is stationary before measuring heading
        stopMotors();

        // Get the heading from the IMU
        double imuHeading = getIMUHeading();

        // Normalize heading to be within 0-360 degrees
        imuHeading = imuHeading % 360.0;
        if (imuHeading < 0.0) {
            imuHeading += 360.0;
        }

        return imuHeading;
    }
    private void setMotorPowerWithRamp(DcMotor motor, double targetPower, double maxSpeed, double acceleration) {
        double currentPower = motor.getPower();
        double desiredPower = Math.min(Math.max(targetPower, -1.0), 1.0);
        double deltaPower = desiredPower - currentPower;
        rampRate = Math.abs(acceleration * runtime.milliseconds() / 1000.0);

        if (deltaPower > 0) {
            motor.setPower(Math.min(currentPower + rampRate, desiredPower));
        } else if (deltaPower < 0) {
            motor.setPower(Math.max(currentPower - rampRate, desiredPower));
        } else {
            motor.setPower(desiredPower);
        }
    }
    private void setMotorPowerWithRampUp(DcMotor motor, double targetPower, double maxSpeed, double ramp_up_time, double ramp_down_time) {
        double currentPower = motor.getPower();
        double desiredPower = Math.min(Math.max(targetPower, -1.0), 1.0);
        double deltaPower = desiredPower - currentPower;
        rampRate = Math.abs(1 * runtime.milliseconds() / 1000.0);
        //rampRate = Math.min(desiredPower / ramp_up_time * runtime.milliseconds() / 1000.0, 1.0);;


        if (deltaPower > 0) {
            motor.setPower(Math.min(currentPower + rampRate, desiredPower));
        } else if (deltaPower < 0) {
            motor.setPower(Math.max(currentPower - rampRate, desiredPower));
        } else {
            motor.setPower(desiredPower);
        }

    }
    public double getRampRate(){
        return rampRate;
    }
    public double getmotorPower(DcMotor motor){
        return motor.getPower();
    }


    private void driveMotorPowerWithRamp(DcMotor motor, double forwardPower, double turnPower, double maxSpeed, double acceleration) {
        // Calculate desired power based on forward and turn power
        double desiredPower = Math.min(Math.max(forwardPower + turnPower, -1.0), 1.0);
        setMotorPowerWithRamp(motor, desiredPower, maxSpeed, acceleration);
    }

    public void setArcadePower(double forwardPower, double turnPower) {
        driveMotorPowerWithRamp(frontLeftMotor, forwardPower, turnPower, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
        driveMotorPowerWithRamp(frontRightMotor, forwardPower, -turnPower, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
        driveMotorPowerWithRamp(rearLeftMotor, forwardPower, turnPower, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
        driveMotorPowerWithRamp(rearRightMotor, forwardPower, -turnPower, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
    }

    public void setMecanumPower(double forwardPower, double strafePower, double turnPower) {
        // Calculate individual motor powers based on robot configuration
        setMotorPowerWithRampUp(frontLeftMotor, forwardPower + strafePower + turnPower, MAX_MOTOR_SPEED, RAMP_UP_TIME, RAMP_DOWN_TIME);
        setMotorPowerWithRampUp(frontRightMotor, forwardPower - strafePower - turnPower, MAX_MOTOR_SPEED, RAMP_UP_TIME, RAMP_DOWN_TIME);
        setMotorPowerWithRampUp(rearLeftMotor, forwardPower + strafePower - turnPower, MAX_MOTOR_SPEED, RAMP_UP_TIME, RAMP_DOWN_TIME);
        setMotorPowerWithRampUp(rearRightMotor, forwardPower - strafePower + turnPower, MAX_MOTOR_SPEED, RAMP_UP_TIME, RAMP_DOWN_TIME);
    }


    public void setFieldOrientedPower(double forwardPower, double strafePower, double turnPower) {
        // Get the robot's current heading from the IMU
        double currentHeading = getIMUHeading();

        // Calculate the adjusted movement powers based on field orientation
        double robotAngle = Math.atan2(forwardPower, strafePower) - Math.toRadians(currentHeading);
        double powerMagnitude = Math.hypot(forwardPower, strafePower);

        // Calculate the adjusted powers for field-oriented driving
        double newForwardPower = powerMagnitude * Math.sin(robotAngle);
        double newStrafePower = powerMagnitude * Math.cos(robotAngle);

        // Apply adjusted powers to achieve field-oriented movement
        setMecanumPower(newForwardPower, newStrafePower, turnPower);
    }
    public void fieldDrive(double forward, double strafe, double rotate) {
        // Get IMU heading
        double imuHeading = getIMUHeading();

        // Convert the robot's movement vector from driver-centric to field-centric
        double robotAngle = Math.atan2(forward, strafe) - Math.toRadians(imuHeading);
        double robotSpeed = Math.sqrt(forward * forward + strafe * strafe);

        // Calculate motor powers for field-oriented movement
        final double v1 = robotSpeed * Math.sin(robotAngle + Math.PI / 4) + rotate;
        final double v2 = robotSpeed * Math.cos(robotAngle + Math.PI / 4) - rotate;
        final double v3 = robotSpeed * Math.cos(robotAngle + Math.PI / 4) + rotate;
        final double v4 = robotSpeed * Math.sin(robotAngle + Math.PI / 4) - rotate;

        // Set motor powers using driveMotorPowerWithRamp
        driveMotorPowerWithRamp(frontLeftMotor, v1, v1, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
        driveMotorPowerWithRamp(frontRightMotor, v2, v2, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
        driveMotorPowerWithRamp(rearLeftMotor, v3, v3, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
        driveMotorPowerWithRamp(rearRightMotor, v4, v4, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
    }
    /**
     * Gets the current motor power.
     * @param motor The DC motor to measure.
     * @return The current power value (-1.0 to 1.0).
     */
    public double getMotorPower(DcMotor motor) {
        return motor.getPower();
    }

    /**
     * Moves the robot forward a specified distance using PID control and ramped speed.
     * @paramdistance The distance to move in inches.
     */
    public void moveForwardPID(double targetDistance, double speed) {
        // Determine the total encoder distance needed for each motor
        double totalTargetDistance = Math.abs(targetDistance);
        double targetEncoderTicks = totalTargetDistance * ENCODER_COUNTS_PER_INCH;

        // Initialize PID variables
        double error, integral, derivative, powerLeft, powerRight;
        double previousErrorLeft = 0.0;
        double previousErrorRight = 0.0;
        double integralTermLeft = 0.0;
        double integralTermRight = 0.0;
        double timestamp = runtime.milliseconds();

        // Start moving forward with initial power
        setMotorPowerWithRamp(frontLeftMotor, speed, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
        setMotorPowerWithRamp(frontRightMotor, speed, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
        setMotorPowerWithRamp(rearLeftMotor, speed, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
        setMotorPowerWithRamp(rearRightMotor, speed, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);

        // Keep moving until both motors reach target distance
        while (Math.abs(getEncoderDistance(frontLeftMotor) - targetEncoderTicks) > ERROR_THRESHOLD &&
                Math.abs(getEncoderDistance(frontRightMotor) - targetEncoderTicks) > ERROR_THRESHOLD) {
            // Calculate individual errors for each motor
            errorLeft = targetEncoderTicks - getEncoderDistance(frontLeftMotor);
            errorRight = targetEncoderTicks - getEncoderDistance(frontRightMotor);

            // Update integral terms for each motor
            integralTermLeft += errorLeft * (runtime.milliseconds() - timestamp);
            integralTermRight += errorRight * (runtime.milliseconds() - timestamp);

            // Calculate individual derivatives for each motor
            derivativeLeft = (errorLeft - previousErrorLeft) / (runtime.milliseconds() - timestamp);
            derivativeRight = (errorRight - previousErrorRight) / (runtime.milliseconds() - timestamp);

            // Calculate PID output power for each motor
            powerLeft = speed + Kp * errorLeft + Ki * integralTermLeft + Kd * derivativeLeft;
            powerRight = speed + Kp * errorRight + Ki * integralTermRight + Kd * derivativeRight;

            // Apply ramped speed control to each motor
            setMotorPowerWithRamp(frontLeftMotor, powerLeft, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
            setMotorPowerWithRamp(frontRightMotor, powerRight, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
            setMotorPowerWithRamp(rearLeftMotor, powerLeft, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
            setMotorPowerWithRamp(rearRightMotor, powerRight, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);

            // Update previous errors and timestamp
            previousErrorLeft = errorLeft;
            previousErrorRight = errorRight;
            timestamp = runtime.milliseconds();
        }

        // Stop motors when target distance is reached
        stopMotors();
    }



    // Move the robot backward a specified distance using PID control and ramped speed.
    public void moveBackwardPID(double targetDistance, double speed) {
        // Determine the total encoder distance needed for each motor
        double totalTargetDistance = Math.abs(targetDistance);
        double targetEncoderTicks = totalTargetDistance * ENCODER_COUNTS_PER_INCH;

        // Initialize PID variables
        double error, integral, derivative, powerLeft, powerRight;
        double previousErrorLeft = 0.0;
        double previousErrorRight = 0.0;
        double integralTermLeft = 0.0;
        double integralTermRight = 0.0;
        double timestamp = runtime.milliseconds();

        // Start moving backwards with initial power
        setMotorPowerWithRamp(frontLeftMotor, -speed, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
        setMotorPowerWithRamp(frontRightMotor, -speed, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
        setMotorPowerWithRamp(rearLeftMotor, -speed, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
        setMotorPowerWithRamp(rearRightMotor, -speed, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);

        // Keep moving until both motors reach target distance
        while (Math.abs(getEncoderDistance(frontLeftMotor) - targetEncoderTicks) > ERROR_THRESHOLD &&
                Math.abs(getEncoderDistance(frontRightMotor) - targetEncoderTicks) > ERROR_THRESHOLD) {
            // Calculate individual errors for each motor
            errorLeft = targetEncoderTicks - getEncoderDistance(frontLeftMotor);
            errorRight = targetEncoderTicks - getEncoderDistance(frontRightMotor);

            // Update integral terms for each motor
            integralTermLeft += errorLeft * (runtime.milliseconds() - timestamp);
            integralTermRight += errorRight * (runtime.milliseconds() - timestamp);

            // Calculate individual derivatives for each motor
            derivativeLeft = (errorLeft - previousErrorLeft) / (runtime.milliseconds() - timestamp);
            derivativeRight = (errorRight - previousErrorRight) / (runtime.milliseconds() - timestamp);

            // Calculate PID output power for each motor
            powerLeft = -speed + Kp * errorLeft + Ki * integralTermLeft + Kd * derivativeLeft;
            powerRight = -speed + Kp * errorRight + Ki * integralTermRight + Kd * derivativeRight;

            // Apply ramped speed control to each motor
            setMotorPowerWithRamp(frontLeftMotor, powerLeft, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
            setMotorPowerWithRamp(frontRightMotor, powerRight, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
            setMotorPowerWithRamp(rearLeftMotor, powerLeft, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
            setMotorPowerWithRamp(rearRightMotor, powerRight, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);

            // Update previous errors and timestamp
            previousErrorLeft = errorLeft;
            previousErrorRight = errorRight;
            timestamp = runtime.milliseconds();
        }

        // Stop motors when target distance is reached
        stopMotors();
    }


    // Strafe the robot sideways a specified distance using PID control and ramped speed.
// Direction: 1 for right, -1 for left
    public void strafePID(double targetDistance, double speed, double strafeAngle) {
        // Convert target distance to encoder ticks based on robot geometry and strafe angle
        double targetEncoderTicks = targetDistance * ENCODER_COUNTS_PER_INCH / Math.sin(strafeAngle);

        // Calculate direction factors based on strafe angle
        double leftDirectionFactor = Math.cos(strafeAngle);
        double rightDirectionFactor = Math.sin(strafeAngle);

        // Initialize PID variables for each motor
        double errorLeft, errorRight, integralLeft, integralRight, derivativeLeft, derivativeRight;
        double previousErrorLeft = 0.0, previousErrorRight = 0.0;
        double integralTermLeft = 0.0, integralTermRight = 0.0;
        double timestamp = runtime.milliseconds();

        // Start strafing with initial power
        setMotorPowerWithRamp(
                frontLeftMotor, speed * leftDirectionFactor, MAX_MOTOR_SPEED, MOTOR_ACCELERATION
        );
        setMotorPowerWithRamp(
                frontRightMotor, speed * rightDirectionFactor, MAX_MOTOR_SPEED, MOTOR_ACCELERATION
        );
        setMotorPowerWithRamp(
                rearLeftMotor, speed * leftDirectionFactor, MAX_MOTOR_SPEED, MOTOR_ACCELERATION
        );
        setMotorPowerWithRamp(
                rearRightMotor, speed * rightDirectionFactor, MAX_MOTOR_SPEED, MOTOR_ACCELERATION
        );

        // Keep moving until both motors reach target distance
        while (
                Math.abs(getEncoderDistance(frontLeftMotor) - targetEncoderTicks) > ERROR_THRESHOLD &&
                        Math.abs(getEncoderDistance(frontRightMotor) - targetEncoderTicks) > ERROR_THRESHOLD
        ) {
            // Calculate individual errors for each motor
            errorLeft = targetEncoderTicks - getEncoderDistance(frontLeftMotor);
            errorRight = targetEncoderTicks - getEncoderDistance(frontRightMotor);

            // Update integral terms for each motor
            integralTermLeft += errorLeft * (runtime.milliseconds() - timestamp);
            integralTermRight += errorRight * (runtime.milliseconds() - timestamp);

            // Calculate individual derivatives for each motor
            derivativeLeft = (errorLeft - previousErrorLeft) / (runtime.milliseconds() - timestamp);
            derivativeRight = (errorRight - previousErrorRight) / (runtime.milliseconds() - timestamp);

            // Calculate PID output power for each motor
            double powerLeft = speed * leftDirectionFactor + Kp * errorLeft + Ki * integralTermLeft + Kd * derivativeLeft;
            double powerRight = speed * rightDirectionFactor + Kp * errorRight + Ki * integralTermRight + Kd * derivativeRight;

            // Apply ramped speed control to each motor
            setMotorPowerWithRamp(
                    frontLeftMotor, powerLeft, MAX_MOTOR_SPEED, MOTOR_ACCELERATION
            );
            setMotorPowerWithRamp(
                    frontRightMotor, powerRight, MAX_MOTOR_SPEED, MOTOR_ACCELERATION
            );
            setMotorPowerWithRamp(
                    rearLeftMotor, powerLeft, MAX_MOTOR_SPEED, MOTOR_ACCELERATION
            );
            setMotorPowerWithRamp(
                    rearRightMotor, powerRight, MAX_MOTOR_SPEED, MOTOR_ACCELERATION
            );

            // Update previous errors and timestamp
            previousErrorLeft = errorLeft;
            previousErrorRight = errorRight;
            timestamp = runtime.milliseconds();
        }

        // Stop motors when target distance is reached
        stopMotors();
    }


    // Turn the robot to a specific angle using PID control and ramped speed.
// Direction: 1 for clockwise, -1 for counter-clockwise
    public void turnPID(double targetAngle, double speed, boolean clockwise) {
        // Convert target angle to encoder ticks based on robot geometry and wheel diameter
        double targetEncoderTicks = Math.abs(targetAngle) * ENCODER_COUNTS_PER_DEGREE;

        // Calculate direction factors based on rotation direction
        double leftDirectionFactor = clockwise ? -1.0 : 1.0;
        double rightDirectionFactor = clockwise ? 1.0 : -1.0;

        // Initialize PID variables
        double error, integral, derivative, powerLeft, powerRight;
        double previousError = 0.0;
        double integralTerm = 0.0;
        double timestamp = runtime.milliseconds();

        // Start turning with initial power
        setMotorPowerWithRamp(
                frontLeftMotor, speed * leftDirectionFactor, MAX_MOTOR_SPEED, MOTOR_ACCELERATION
        );
        setMotorPowerWithRamp(
                frontRightMotor, speed * rightDirectionFactor, MAX_MOTOR_SPEED, MOTOR_ACCELERATION
        );
        setMotorPowerWithRamp(
                rearLeftMotor, speed * leftDirectionFactor, MAX_MOTOR_SPEED, MOTOR_ACCELERATION
        );
        setMotorPowerWithRamp(
                rearRightMotor, speed * rightDirectionFactor, MAX_MOTOR_SPEED, MOTOR_ACCELERATION
        );

        // Keep turning until the target angle is reached
        while (Math.abs(getAverageEncoderDistance()) < targetEncoderTicks) {
            // Calculate average encoder distance
            double averageEncoderDistance = getAverageEncoderDistance();

            // Calculate error based on target and measured distance
            error = targetEncoderTicks - averageEncoderDistance;

            // Update integral term
            integralTerm += error * (runtime.milliseconds() - timestamp);

            // Calculate derivative
            derivative = (error - previousError) / (runtime.milliseconds() - timestamp);

            // Calculate PID output power for each motor
            double linearPower = Kp * error + Ki * integralTerm + Kd * derivative;
            powerLeft = speed * leftDirectionFactor + linearPower;
            powerRight = speed * rightDirectionFactor + linearPower;

            // Apply ramped speed control to each motor
            setMotorPowerWithRamp(
                    frontLeftMotor, powerLeft, MAX_MOTOR_SPEED, MOTOR_ACCELERATION
            );
            setMotorPowerWithRamp(
                    frontRightMotor, powerRight, MAX_MOTOR_SPEED, MOTOR_ACCELERATION
            );
            setMotorPowerWithRamp(
                    rearLeftMotor, powerLeft, MAX_MOTOR_SPEED, MOTOR_ACCELERATION
            );
            setMotorPowerWithRamp(
                    rearRightMotor, powerRight, MAX_MOTOR_SPEED, MOTOR_ACCELERATION
            );

            // Update previous error and timestamp
            previousError = error;
            timestamp = runtime.milliseconds();
        }

        // Stop motors when target angle is reached
        stopMotors();
    }


// ... Add methods for stopping motors, setting motor mode, etc. ...
    public double getEncoderDistance(DcMotor motor) {
        // Get individual encoder distance for specified motor
        if (motor.equals(frontLeftMotor)) {
            return frontLeftMotor.getCurrentPosition();
        } else if (motor.equals(frontRightMotor)) {
            return frontRightMotor.getCurrentPosition();
        } else if (motor.equals(rearLeftMotor)) {
            return rearLeftMotor.getCurrentPosition();
        } else if (motor.equals(rearRightMotor)) {
            return rearRightMotor.getCurrentPosition();
        } else {return 0;}
    }
    // Stop all motors
    public void stopMotors() {
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        rearLeftMotor.setPower(0.0);
        rearRightMotor.setPower(0.0);
    }

    // Set the motor mode for all motors
    public void setMotorMode(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        rearLeftMotor.setMode(mode);
        rearRightMotor.setMode(mode);
    }

    public double encoderTicksToInches(double encoderTicks) {
        return encoderTicks / ENCODER_COUNTS_PER_INCH;
    }

    public double inchesToEncoderTicks(double distance) {
        return distance * ENCODER_COUNTS_PER_INCH;
    }

    public void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure encoders are in RUN_USING_ENCODER mode before resuming operation
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getAverageEncoderPosition() {
        double leftEncoderPosition = frontLeftMotor.getCurrentPosition() + rearLeftMotor.getCurrentPosition();
        double rightEncoderPosition = frontRightMotor.getCurrentPosition() + rearRightMotor.getCurrentPosition();
        return (leftEncoderPosition + rightEncoderPosition) / 2;
    }
    private double getAverageEncoderDistance() {
        // Get individual encoder distances for each motor
        double leftDistance = getEncoderDistance(frontLeftMotor);
        double rightDistance = getEncoderDistance(frontRightMotor);
        double rearLeftDistance = getEncoderDistance(rearLeftMotor);
        double rearRightDistance = getEncoderDistance(rearRightMotor);

        // Calculate average encoder distance
        double averageDistance = (leftDistance + rightDistance + rearLeftDistance + rearRightDistance) / 4.0;

        return averageDistance;
    }


    public double getAverageEncoderAngle() {
        // Get average encoder position
        double averagePosition = getAverageEncoderPosition();

        // Convert average position to degrees
        double angle = averagePosition / ENCODER_COUNTS_PER_DEGREE;

        return angle;
    }

    public void moveArcPID(double radius, double angle, DcMotor directionMotor) {
        // Validate inputs
        if (radius <= 0 || angle <= 0 || angle > 360) {
            throw new IllegalArgumentException("Invalid parameters for moveArcPID.");
        }

        // Convert angle to encoder ticks
        double encoderTicks = angle * ENCODER_COUNTS_PER_DEGREE;

        // Initialize PID variables
        double error, integral, derivative, powerLeft, powerRight;
        double previousError = 0.0;
        double integralTerm = 0.0;
        double timestamp = runtime.milliseconds();

        // Start moving with low initial power
        setMotorPowerWithRamp(frontLeftMotor, 0.2, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
        setMotorPowerWithRamp(frontRightMotor, 0.2, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
        setMotorPowerWithRamp(rearLeftMotor, 0.2, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
        setMotorPowerWithRamp(rearRightMotor, 0.2, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);

        // Loop until target angle is reached for the specific motor
        while (Math.abs(getEncoderDistance(directionMotor)) < encoderTicks) {
            // Calculate error and update integral term for the specific motor
            error = encoderTicks - getEncoderDistance(directionMotor);
            integralTerm += error * (runtime.milliseconds() - timestamp);

            // Calculate derivative
            derivative = (error - previousError) / (runtime.milliseconds() - timestamp);

            // Calculate PID output power for each motor based on the direction motor
            double linearPower = Kp * error + Ki * integralTerm + Kd * derivative;
            powerLeft = linearPower + (K_ARC * radius) / 2.0;
            powerRight = linearPower - (K_ARC * radius) / 2.0;

            // Apply ramped speed control to each motor
            setMotorPowerWithRamp(frontLeftMotor, powerLeft, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
            setMotorPowerWithRamp(frontRightMotor, powerRight, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
            setMotorPowerWithRamp(rearLeftMotor, powerLeft, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
            setMotorPowerWithRamp(rearRightMotor, powerRight, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);

            // Update previous error and timestamp
            previousError = error;
            timestamp = runtime.milliseconds();
        }

        // Stop motors
        stopMotors();
    }



    private double calculateEncoderCountsPerDegree(double wheelDiameter, int encoderResolution, double gearRatio) {
        // Calculate circumference of the wheel
        double circumference = Math.PI * wheelDiameter;

        // Calculate distance traveled per encoder tick
        double distancePerTick = circumference / encoderResolution * gearRatio;

        // Calculate encoder counts per degree
        return 360 / distancePerTick;
    }
    /*
    public void odometryUpdate(double leftEncoderDistance, double rightEncoderDistance) {
        // Convert encoder distances to robot displacement
        double robotDisplacement = (leftEncoderDistance + rightEncoderDistance) / 2.0;

        // Calculate robot heading based on encoder difference
        double headingChange = (rightEncoderDistance - leftEncoderDistance) / ENCODER_WHEEL_BASE;

        // Update robot pose based on displacement and heading change
        updateRobotPose(robotDisplacement, headingChange);
    }


     */
}