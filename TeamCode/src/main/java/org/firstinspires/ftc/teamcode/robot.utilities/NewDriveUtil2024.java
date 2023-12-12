package org.firstinspires.ftc.teamcode.robot.utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.util.ElapsedTime;

public class NewDriveUtil2024 {

    // Define constants
    private static final double ENCODER_COUNTS_PER_INCH = 50;
    private static final double MAX_MOTOR_SPEED = 312;
    private static final double MOTOR_ACCELERATION = 1;
    private final double ENCODER_COUNTS_PER_DEGREE;
    private static final double WHEEL_DIAMETER = 4.0;

    // PID constants
    private static final double Kp = 0.5;
    private static final double Ki = 0.01;
    private static final double Kd = 0.1;

    // Ramp parameters
    private static final double RAMP_UP_TIME = 0.2; // Ramp-up time in seconds
    private static final double RAMP_DOWN_TIME = 0.1; // Ramp-down time in seconds

    // Class members
    private final DcMotor frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;
    private final ElapsedTime runtime;
    private final double wheelDiameter = 0;
    private final int encoderResolution = 0;
    private final double gearRatio = 0;
    private final double ERROR_THRESHOLD = 10; // Define appropriate threshold based on encoder resolution and desired accuracy
    double errorLeft, errorRight;
    double derivativeLeft, derivativeRight;
    public enum MotorType {
    FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT
}
    public NewDriveUtil2024(HardwareMap hardwareMap) {

        // Initialize motor objects
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "rear_left_motor");
        rearRightMotor = hardwareMap.get(DcMotor.class, "rear_right_motor");

        // Set motor directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        runtime = new ElapsedTime();
        // Define ENCODER_COUNTS_PER_DEGREE based on your robot configuration
        // This depends on factors like wheel diameter, encoder resolution, and gear ratios
     
        ENCODER_COUNTS_PER_DEGREE = calculateEncoderCountsPerDegree(wheelDiameter, encoderResolution, gearRatio);
    }


    private void setMotorPowerWithRamp(DcMotor motor, double power, double maxSpeed, double acceleration) {
        // Calculate ramped power based on target power, max speed, and acceleration
        double desiredPower = Math.min(Math.max(power, -1.0), 1.0);
        double currentPower = motor.getPower();
        double deltaPower = desiredPower - currentPower;
        double rampRate = Math.abs(acceleration * runtime.milliseconds() / 1000.0);

        // Apply ramped power change to avoid sudden jumps
        if (deltaPower > 0) {
            motor.setPower(Math.min(currentPower + rampRate, desiredPower));
        } else if (deltaPower < 0) {
            motor.setPower(Math.max(currentPower - rampRate, desiredPower));
        } else {
            motor.setPower(desiredPower);
        }
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
        }
        throw new IllegalArgumentException("Invalid motor type for getEncoderDistance.");
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
/*
    public double getHeading() {
        // Convert average encoder position to degrees
        double angle = getAverageEncoderAngle();

        // Apply gyro correction if available
        if (gyroSensor != null) {
            angle += gyroSensor.getHeading();
        }

        // Normalize angle to 0-360 degrees
        angle %= 360;

        return angle;
    }
    public double getHeading() {
        // Ensure robot is stationary before measuring heading
        stopMotors();

        // Get average encoder distance from all motors
        double averageEncoderDistance = getAverageEncoderDistance();

        // Calculate heading based on encoder distance and robot geometry
        double heading = (averageEncoderDistance / ENCODER_COUNTS_PER_DEGREE) * GEAR_RATIO * WHEEL_DIAMETER / ROBOT_WHEELBASE * 360.0;

        // Apply compass calibration offset if available
        if (compass != null && compass.isCalibrated()) {
            heading += compass.getHeading();
        }

        // Normalize heading to be within 0-360 degrees
        heading = heading % 360.0;
        if (heading < 0.0) {
            heading += 360.0;
        }

        return heading;
    }
*/
    public double getAverageEncoderAngle() {
        // Get average encoder position
        double averagePosition = getAverageEncoderPosition();

        // Convert average position to degrees
        double angle = averagePosition / ENCODER_COUNTS_PER_DEGREE;

        return angle;
    }
/*
    public void moveArcPID(double radius, double angle, int direction) {
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
        setMotorPowerWithRamp(frontLeftMotor, direction * 0.2);
        setMotorPowerWithRamp(frontRightMotor, direction * 0.2);
        setMotorPowerWithRamp(rearLeftMotor, direction * 0.2);
        setMotorPowerWithRamp(rearRightMotor, direction * 0.2);

        // Loop until target angle is reached
        while (Math.abs(getEncoderDistance(direction)) < encoderTicks) {
            // Calculate error and update integral term
            error = encoderTicks - getEncoderDistance(direction);
            integralTerm += error * (runtime.milliseconds() - timestamp);

            // Calculate derivative
            derivative = (error - previousError) / (runtime.milliseconds() - timestamp);

            // Calculate PID output power for each motor
            double linearPower = Kp * error + Ki * integralTerm + Kd * derivative;
            powerLeft = linearPower + (K_ARC * radius) / 2.0;
            powerRight = linearPower - (K_ARC * radius) / 2.0;

            // Apply ramped speed control to each motor
            setMotorPowerWithRamp(frontLeftMotor, direction * powerLeft);
            setMotorPowerWithRamp(frontRightMotor, direction * powerRight);
            setMotorPowerWithRamp(rearLeftMotor, direction * powerLeft);
            setMotorPowerWithRamp(rearRightMotor, direction * powerRight);

            // Update previous error and timestamp
            previousError = error;
            timestamp = runtime.milliseconds();
        }

        // Stop motors
        stopMotors();
    }

 */
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