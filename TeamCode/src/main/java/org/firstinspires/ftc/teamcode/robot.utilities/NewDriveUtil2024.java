    package org.firstinspires.ftc.teamcode.robot.utilities;


    import static android.os.SystemClock.sleep;

    import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import com.qualcomm.robotcore.util.ElapsedTime;
    import com.qualcomm.robotcore.hardware.IMU;

    import org.firstinspires.ftc.robotcore.external.Telemetry;
    import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
    import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
    import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
    import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
    import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
    import org.firstinspires.ftc.teamcode.robot.utilities.Tests.TestTeleOpMove;

    import com.qualcomm.robotcore.util.Range;
    import com.qualcomm.robotcore.util.RobotLog;

    public class NewDriveUtil2024 {
        private LinearOpMode myOpMode = null;
        // Define constants
        private static final double ENCODER_COUNTS_PER_INCH = 45.33;
        //Diameter = 96mm = 3.77953 inches (approximately)
        // Circumference = π * Diameter Circumference = π * 3.77953 inches ≈ 11.87 inches
        //Counts per inch = Encoder Resolution / Circumference
        //Counts per inch = 537.7 / 11.87 ≈ 45.33
        private double ENCODER_COUNTS_PER_DEGREE; //SET IN CONSTRUCTOR //537.7 / 360; //≈ 1.4931;
        private final double WHEEL_DIAMETER_INCHES = 3.77953;
        private static final int ENCODER_RESOLUTION = 537;
        private final double GEAR_RATIO = 1;
        private static final double MAX_MOTOR_SPEED = .8;
        private static final double MOTOR_ACCELERATION = 1;

        static final double WHEEL_DIAMETER_CM = 9.6;     // In centimeters
        static final double WHEEL_RADIUS = WHEEL_DIAMETER_CM /2; // in cm
        static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_CM * Math.PI;
        static final double GEAR_REDUCTION = 1.0; // output (wheel) speed / input (motor) speed
        public static double TRACK_WIDTH = 16.45; // in
        static final double COUNTS_PER_GEAR_REV = ENCODER_RESOLUTION * GEAR_REDUCTION;
        static final double COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV/360;

        // PID constants
        private static final double Kp = 0.005;
        private static final double Ki = 0.00;
        private static final double Kd = 0.00;

        // Ramp parameters
        private static final double RAMP_UP_TIME = 0.2; // Ramp-up time in seconds
        private static final double RAMP_DOWN_TIME = 0.2; // Ramp-down time in seconds
        private static final double K_ARC = 0.1; // Adjust the value according to your requirement


        // Class members
        public DcMotor left_front_motor;
        public DcMotor right_front_motor;
        public DcMotor left_rear_motor;
        public DcMotor right_rear_motor;
        private ElapsedTime runtime;

        private final double ERROR_THRESHOLD = 10; // Define appropriate threshold based on encoder resolution and desired accuracy
        double errorLeft, errorRight;
        double derivativeLeft, derivativeRight;
        private double rampRate;
        static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
        static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable
        static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
        // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
        // Define the Proportional control coefficient (or GAIN) for "heading control".
        // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
        // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
        // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)

        public enum motors {
            frontLeftMotor,
            frontRightMotor,
            rearLeftMotor,
            rearRightMotor
        }
        public enum UnitOfMeasure {
            INCHES,
            CM
        }

        // Assuming you have an IMU object initialized
        private IMU imu;
        private Telemetry telemetry;

        /* local OpMode members. */
        HardwareMap hardwareMap =  null;


      //  public NewDriveUtil2024(HardwareMap hardwareMap, Telemetry telemetry) {
       // }

        public NewDriveUtil2024(LinearOpMode opmode) {
            myOpMode = opmode;
        }
        //drive utility constructor
        public void init(HardwareMap ahwMap, Telemetry telemetry ) {
            // Save reference to Hardware map
            hardwareMap = ahwMap;

            this.telemetry = telemetry;
            // Initialize motor objects
            left_front_motor = hardwareMap.get(DcMotor.class, "Front_Left");
            right_front_motor = hardwareMap.get(DcMotor.class, "Front_Right");
            left_rear_motor = hardwareMap.get(DcMotor.class, "Rear_Left");
            right_rear_motor = hardwareMap.get(DcMotor.class, "Rear_Right");

            // Set motor directions
            left_front_motor.setDirection(DcMotorSimple.Direction.REVERSE);
            left_rear_motor.setDirection(DcMotorSimple.Direction.REVERSE);

            //Set brake condition to full stop (allso can give DcMotor.ZeroPowerBehavior.FLOAT)
            left_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_rear_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_rear_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            runtime = new ElapsedTime();
            // Define ENCODER_COUNTS_PER_DEGREE based on your robot configuration
            // This depends on factors like wheel diameter, encoder resolution, and gear ratios
            ENCODER_COUNTS_PER_DEGREE = calculateEncoderCountsPerDegree(WHEEL_DIAMETER_CM, ENCODER_RESOLUTION, GEAR_RATIO);

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
            return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY,AngleUnit.DEGREES).firstAngle;
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
       /* private void setMotorPowerWithRamp(DcMotor motor, double targetPower, double maxSpeed, double acceleration) {
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
        */

        public double getRampRate(){
            return rampRate;
        }
        public double getmotorPower(DcMotor motor){
            /* tested */
            return motor.getPower();
        }
        public void driveMecanum(double forwardPower, double strafePower, double turnPower, double rightStickY, double driveSpeed) {
            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            StackTraceElement[] stacktrace = Thread.currentThread().getStackTrace();
            StackTraceElement e = stacktrace[3];//maybe this number needs to be corrected
            RobotLog.dd("GAMLOG", "current method: "+name+": called from: "+e);

            double y = forwardPower * driveSpeed; // Remember, this is reversed!
            double x = strafePower * 1.1 * driveSpeed; // Counteract imperfect strafing
            double rx = turnPower * driveSpeed;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            DcMotor[] motors = {
                    left_front_motor,
                    right_front_motor,
                    left_rear_motor,
                    right_rear_motor
            };
            double[] powers = {
                    (y + x + rx) / denominator,
                    (y - x - rx) / denominator,
                    (y - x + rx) / denominator,
                    (y + x - rx) / denominator
            };

            setMotorPowersWithRamp(motors, powers, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
        }

        public void driveFieldOriented(double forwardPower, double strafePower, double turnPower, double rightStickY, double driveSpeed) {
            /** TESTED **/
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = strafePower * Math.cos(-botHeading) - forwardPower * Math.sin(-botHeading);
            double rotY = strafePower * Math.sin(-botHeading) + forwardPower * Math.cos(-botHeading);
            double rx = turnPower;
            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            DcMotor[] motors = {
                    left_front_motor,
                    right_front_motor,
                    left_rear_motor,
                    right_rear_motor};
            double[] powers = {
                    (rotY + rotX + rx) / denominator,
                    (rotY - rotX - rx) / denominator,
                    (rotY - rotX + rx) / denominator,
                    (rotY + rotX - rx) / denominator };

            telemetry.addData("front left Motor:   ",forwardPower + strafePower + turnPower);
            telemetry.addData("frontRightMotor:   ",forwardPower - strafePower - turnPower);
            telemetry.addData("rearLeftMotor:   ",forwardPower - strafePower + turnPower);
            telemetry.addData("rearRightMotor:   ",forwardPower + strafePower - turnPower);
            telemetry.addData("rotX: ", rotX);
            telemetry.addData("rotY: ", rotY);
            telemetry.addData("botHeading new: ", botHeading);
            // Retrieve Rotational Angles and Velocities
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.RADIANS));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);

            setMotorPowersWithRamp(motors, powers, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
        }

        /* this method was provide from first as a sample method in RobotAutoDriveToAprilTagOmni */
        public void moveRobot(double x, double y, double yaw) {
            // Calculate wheel powers.
            double leftFrontPower    =  x -y -yaw;
            double rightFrontPower   =  x +y +yaw;
            double leftBackPower     =  x +y -yaw;
            double rightBackPower    =  x -y +yaw;

            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send powers to the wheels.
            left_front_motor.setPower(leftFrontPower);
            right_front_motor.setPower(rightFrontPower);
            left_rear_motor.setPower(leftBackPower);
            right_rear_motor.setPower(rightBackPower);
        }
        public enum Direction {
            FORWARD,
            BACKWARD,
            LEFT,
            RIGHT
        }

        public void movePID(Direction direction, double targetDistance, double speed, long timeoutMillis) {

            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            StackTraceElement[] stacktrace = Thread.currentThread().getStackTrace();
            StackTraceElement e = stacktrace[3];//maybe this number needs to be corrected
            RobotLog.dd("GAMLOG", "current method: "+name+": called from: "+e);

            DcMotor[] motors = {left_front_motor, right_front_motor, left_rear_motor, right_rear_motor};
            double countsToTravel = targetDistance * ENCODER_COUNTS_PER_INCH;

            double initialPosition = getAverageEncoderPosition(motors);

            double errorSum = 0;
            double lastError = 0;
            long startTime = System.currentTimeMillis();


            while ((System.currentTimeMillis() - startTime) < timeoutMillis &&
                    Math.abs(getAverageEncoderPosition(motors) - initialPosition) < countsToTravel) {
                double error = countsToTravel - (getAverageEncoderPosition(motors) - initialPosition);
                errorSum += error;
                double dError = error - lastError;

                double power = ((error * Kp) + (errorSum * Ki) + (dError * Kd));

                if (direction == Direction.BACKWARD || direction == Direction.LEFT) {
                    power = -power; // Reverse power for backward and left movements
                }

                if (direction == Direction.LEFT || direction == Direction.RIGHT) {
                    // For left and right movements, adjust power based on individual motors
                    double leftPower = power;
                    double rightPower = -power; // Invert right power for strafing right

                    if (direction == Direction.LEFT) {
                        double temp = leftPower;
                        leftPower = rightPower;
                        rightPower = temp;
                    }

                    double[] powers = {
                            leftPower * calculateIndividualMotorCorrection(left_front_motor),
                            rightPower * calculateIndividualMotorCorrection(right_front_motor),
                            rightPower * calculateIndividualMotorCorrection(left_rear_motor),
                            leftPower * calculateIndividualMotorCorrection(right_rear_motor)
                    };

                    setMotorPowersWithRamp(motors, powers, speed, MOTOR_ACCELERATION);
                } else {
                    // For forward and backward movements, apply power to all motors
                    for (DcMotor motor : motors) {
                        double individualMotorCorrection = calculateIndividualMotorCorrection(motor);
                        setMotorPowerWithRamp(motor, power * individualMotorCorrection, speed, MOTOR_ACCELERATION);
                    }
                }

                lastError = error;
                RobotLog.dd("GAMLOG-DEBUG-"+name,
                        "initial position="+initialPosition
                                +";"+"goal counts to travel="+countsToTravel
                                +";"+"Timeout="+timeoutMillis
                                +";"+"elapsed time="+String.valueOf((System.currentTimeMillis() - startTime))
                                +";"+"power="+power
                                +";"+"error="+error
                                +";"+"Kp="+Kp
                                +";"+"errorsum="+errorSum
                                +";"+"Ki="+Ki
                                +";"+"dError="+dError
                                +";"+"Kd="+Kd
                                +";"+"average motor position="+getAverageEncoderPosition(motors)
                                +";"+"motor0 position="+motors[0].getCurrentPosition()
                                +";"+"motor1 position="+motors[1].getCurrentPosition()
                                +";"+"motor2 position="+motors[2].getCurrentPosition()
                                +";"+"motor3 position="+motors[3].getCurrentPosition()
                );
            }

            stopMotors(motors); // Stop the motors after reaching the target distance or timeout
        }

        public void moveForwardPID(double targetDistance, double speed, long timeoutMillis) {
            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            StackTraceElement[] stacktrace = Thread.currentThread().getStackTrace();
            StackTraceElement e = stacktrace[3];//maybe this number needs to be corrected
            RobotLog.dd("GAMLOG", "current method: "+name+": called from: "+e);

            DcMotor[] motors = {left_front_motor, right_front_motor, left_rear_motor, right_rear_motor};
            double countsToTravel = targetDistance * ENCODER_COUNTS_PER_INCH;

            // Calculate the average initial position among all motors
            double initialPosition = getAverageEncoderPosition(motors);

            double errorSum = 0;
            double lastError = 0;
            long startTime = System.currentTimeMillis();

            while ((System.currentTimeMillis() - startTime) < timeoutMillis &&
                    Math.abs(getAverageEncoderPosition(motors) - initialPosition) < countsToTravel) {
                // Calculate the average error among all motors
                double error = countsToTravel - (getAverageEncoderPosition(motors) - initialPosition);
                errorSum += error;
                double dError = error - lastError;

                double power = (error * Kp) + (errorSum * Ki) + (dError * Kd);

                //RobotLog.dd("GAMLOG-DEBUG-"+name,"initialPosition = "+initialPosition);
                //RobotLog.dd("GAMLOG-DEBUG-"+name,"countsToTravel = "+countsToTravel);
                //RobotLog.dd("GAMLOG-DEBUG-"+name,"ERROR = "+error);
                //RobotLog.dd("GAMLOG-DEBUG-"+name,"powercalculation = "+power);
                RobotLog.dd("GAMLOG-DEBUG-"+name,
                        "initial position="+initialPosition
                        +";"+"goal counts to travel="+countsToTravel
                        +";"+"Timeout="+timeoutMillis
                        +";"+"elapsed time="+String.valueOf((System.currentTimeMillis() - startTime))
                        +";"+"power="+power
                        +";"+"error="+error
                        +";"+"average motor position="+getAverageEncoderPosition(motors)
                        +";"+"motor0 position="+motors[0].getCurrentPosition()
                        +";"+"motor1 position="+motors[1].getCurrentPosition()
                        +";"+"motor2 position="+motors[2].getCurrentPosition()
                        +";"+"motor3 position="+motors[3].getCurrentPosition()
                );
                //RobotLog.d("GAMLOG - timer "+ ((System.currentTimeMillis() - startTime)));
               // RobotLog.dd("GAMLOG-DEBUG-"+name, "timer is set to: "+String.valueOf((System.currentTimeMillis() - startTime)));
                //RobotLog.dd("GAMLOG-DEBUG-"+name,"elapsed timeout "+ timeoutMillis);
                //RobotLog.dd("GAMLOG-DEBUG-"+name, "motor0 position = "+motors[0].getCurrentPosition());
                //RobotLog.dd("GAMLOG-DEBUG-"+name, "motor1 position = "+motors[1].getCurrentPosition());
                //RobotLog.dd("GAMLOG-DEBUG-"+name, "motor2 position = "+motors[2].getCurrentPosition());
                //RobotLog.dd("GAMLOG-DEBUG-"+name, "motor3 position = "+motors[3].getCurrentPosition());


                for (DcMotor motor : motors) {
                    // Adjust power based on individual motor corrections if needed
                    double individualMotorCorrection = calculateIndividualMotorCorrection(motor);
                    RobotLog.dd("GAMLOG-DEBUG-"+name,"individualMotorCorrection for "+motor+"= "+individualMotorCorrection
                    +";"+"attempt to set power to= "+power * individualMotorCorrection);
                    setMotorPowerWithRamp(motor, power * individualMotorCorrection, speed, MOTOR_ACCELERATION);
                    //setMotorPowerWithRamp(motor, power, .5, MOTOR_ACCELERATION);

                }

                lastError = error;
            }

            stopMotors(motors); // Stop the motors after reaching the target distance or timeout
        }
        private void setMotorPowersWithRamp(DcMotor[] motors, double[] powers, double maxSpeed, double acceleration) {

            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            StackTraceElement[] stacktrace = Thread.currentThread().getStackTrace();
            StackTraceElement e = stacktrace[3];//maybe this number needs to be corrected
            RobotLog.dd("GAMLOG", "current method: "+name+": called from: "+e);

            double[] desiredPowers = new double[motors.length];
            for (int i = 0; i < motors.length; i++) {
                desiredPowers[i] = Math.min(Math.max(powers[i], -1.0), 1.0);
                setMotorPowerWithRamp(motors[i], desiredPowers[i], maxSpeed, acceleration);
            }
        }
        private void setMotorPowerWithRamp(DcMotor motor, double targetPower, double maxSpeed, double acceleration) {

            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            StackTraceElement[] stacktrace = Thread.currentThread().getStackTrace();
            StackTraceElement e = stacktrace[3];//maybe this number needs to be corrected
            RobotLog.dd("GAMLOG", "current method: "+name+": called from: "+e);

            double currentPower = motor.getPower();
            double desiredPower = Math.min(Math.max(targetPower, -1.0), 1.0);
            double deltaPower = desiredPower - currentPower;

            double rampRate = Math.abs(acceleration * runtime.milliseconds() / 1000.0);
            double powerChange = deltaPower; // Calculate change in power

            double newPower = currentPower + powerChange;

            // Ensure new power is within the bounds of -maxSpeed and maxSpeed
            newPower = Math.min(Math.max(newPower, -maxSpeed), maxSpeed);
            RobotLog.dd("GAMLOG", "current method: "+name+" start of telemetry");
            RobotLog.dd("GAMLOG","current power: "+currentPower);
            RobotLog.dd("GAMLOG", "desired power: "+desiredPower);
            RobotLog.dd("GAMLOG","delta power: "+deltaPower);
            RobotLog.dd("GAMLOG","new Power: "+newPower);
            RobotLog.dd("GAMLOG", "position of motor: "+motor.getCurrentPosition());
            RobotLog.dd("GAMLOG", "current power of motor power to set: "+motor.getPower());
            RobotLog.dd("GAMLOG", "current method: "+name+"end of telemetry");
            motor.setPower(newPower);
        }

        private double getAverageEncoderPosition(DcMotor[] motors) {
            int total = 0;
            for (DcMotor motor : motors) {
                total += motor.getCurrentPosition();
            }
            return total / motors.length;
        }

        private double calculateIndividualMotorCorrection(DcMotor motor) {
            // Implement individual motor correction calculation based on differences in encoder readings
            // This could involve applying a correction factor or adjustment specific to each motor
            // Return a value to adjust the power for each motor
            if (motor == left_front_motor){
                return 1;//1.060986547;
            }
            else if (motor == right_front_motor){
                return 1.00;
            }
            else if (motor == left_rear_motor){
                return 1;//1.053428317;
            }else if (motor == right_rear_motor) {
                return 1;//1.043209877;
            }else return 1.0;
        }

        // Method to stop all motors
        private void stopMotors(DcMotor[] motors) {
            for (DcMotor motor : motors) {
                motor.setPower(0);
            }
        }
        /*
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

         */

        public void moveBackwardPID(double targetDistance, double speed, long timeoutMillis) {
            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            StackTraceElement[] stacktrace = Thread.currentThread().getStackTrace();
            StackTraceElement e = stacktrace[3];//maybe this number needs to be corrected
            RobotLog.dd("GAMLOG", "current method: "+name+": called from: "+e);

            DcMotor[] motors = {left_front_motor, right_front_motor, left_rear_motor, right_rear_motor};
            double countsToTravel = targetDistance * ENCODER_COUNTS_PER_INCH;

            // Calculate the average initial position among all motors
            double initialPosition = getAverageEncoderPosition(motors);

            double errorSum = 0;
            double lastError = 0;
            long startTime = System.currentTimeMillis();

            while ((System.currentTimeMillis() - startTime) < timeoutMillis &&
                    Math.abs(getAverageEncoderPosition(motors) - initialPosition) < countsToTravel) {
                // Calculate the average error among all motors
                double error = countsToTravel - (getAverageEncoderPosition(motors) - initialPosition);
                errorSum += error;
                double dError = error - lastError;

                double power = ((error * Kp) + (errorSum * Ki) + (dError * Kd));
                power = -power;

                //RobotLog.dd("GAMLOG-DEBUG-"+name,"initialPosition = "+initialPosition);
                //RobotLog.dd("GAMLOG-DEBUG-"+name,"countsToTravel = "+countsToTravel);
                //RobotLog.dd("GAMLOG-DEBUG-"+name,"ERROR = "+error);
                //RobotLog.dd("GAMLOG-DEBUG-"+name,"powercalculation = "+power);
                RobotLog.dd("GAMLOG-DEBUG-"+name,
                        "initial position="+initialPosition
                                +";"+"goal counts to travel="+countsToTravel
                                +";"+"Timeout="+timeoutMillis
                                +";"+"elapsed time="+String.valueOf((System.currentTimeMillis() - startTime))
                                +";"+"power="+power
                                +";"+"error="+error
                                +";"+"average motor position="+getAverageEncoderPosition(motors)
                                +";"+"motor0 position="+motors[0].getCurrentPosition()
                                +";"+"motor1 position="+motors[1].getCurrentPosition()
                                +";"+"motor2 position="+motors[2].getCurrentPosition()
                                +";"+"motor3 position="+motors[3].getCurrentPosition()
                );
                //RobotLog.d("GAMLOG - timer "+ ((System.currentTimeMillis() - startTime)));
                // RobotLog.dd("GAMLOG-DEBUG-"+name, "timer is set to: "+String.valueOf((System.currentTimeMillis() - startTime)));
                //RobotLog.dd("GAMLOG-DEBUG-"+name,"elapsed timeout "+ timeoutMillis);
                //RobotLog.dd("GAMLOG-DEBUG-"+name, "motor0 position = "+motors[0].getCurrentPosition());
                //RobotLog.dd("GAMLOG-DEBUG-"+name, "motor1 position = "+motors[1].getCurrentPosition());
                //RobotLog.dd("GAMLOG-DEBUG-"+name, "motor2 position = "+motors[2].getCurrentPosition());
                //RobotLog.dd("GAMLOG-DEBUG-"+name, "motor3 position = "+motors[3].getCurrentPosition());


                for (DcMotor motor : motors) {
                    // Adjust power based on individual motor corrections if needed
                    double individualMotorCorrection = calculateIndividualMotorCorrection(motor);
                    RobotLog.dd("GAMLOG-DEBUG-"+name,"individualMotorCorrection for "+motor+"= "+individualMotorCorrection
                            +";"+"attempt to set power to= "+power * individualMotorCorrection);
                    setMotorPowerWithRamp(motor, power * individualMotorCorrection, speed, MOTOR_ACCELERATION);
                    //setMotorPowerWithRamp(motor, power, .5, MOTOR_ACCELERATION);

                }

                lastError = error;
            }

            stopMotors(motors); // Stop the motors after reaching the target distance or timeout
        }

        public void strafeLeftPID(double targetDistance, double speed, long timeoutMillis) {
            DcMotor[] motors = {left_front_motor, right_front_motor, left_rear_motor, right_rear_motor};
            double countsToTravel = targetDistance * ENCODER_COUNTS_PER_INCH;

            double initialPosition = getAverageEncoderPosition(motors);

            double errorSum = 0;
            double lastError = 0;
            long startTime = System.currentTimeMillis();

            while ((System.currentTimeMillis() - startTime) < timeoutMillis &&
                    Math.abs(getAverageEncoderPosition(motors) - initialPosition) < countsToTravel) {
                double error = countsToTravel - (getAverageEncoderPosition(motors) - initialPosition);
                errorSum += error;
                double dError = error - lastError;

                double leftPower = ((error * Kp) + (errorSum * Ki) + (dError * Kd));
                leftPower = -leftPower; // Reverse for left strafe

                double rightPower = ((error * Kp) + (errorSum * Ki) + (dError * Kd));

                // Apply individual motor corrections
                double[] powers = {
                        leftPower * calculateIndividualMotorCorrection(left_front_motor),
                        rightPower * calculateIndividualMotorCorrection(right_front_motor),
                        rightPower * calculateIndividualMotorCorrection(left_rear_motor),
                        leftPower * calculateIndividualMotorCorrection(right_rear_motor)
                };

                setMotorPowersWithRamp(motors, powers, speed, MOTOR_ACCELERATION);

                lastError = error;
            }

            stopMotors(motors);
        }

        public void strafeRightPID(double targetDistance, double speed, long timeoutMillis) {
            DcMotor[] motors = {left_front_motor, right_front_motor, left_rear_motor, right_rear_motor};
            double countsToTravel = targetDistance * ENCODER_COUNTS_PER_INCH;

            double initialPosition = getAverageEncoderPosition(motors);

            double errorSum = 0;
            double lastError = 0;
            long startTime = System.currentTimeMillis();

            while ((System.currentTimeMillis() - startTime) < timeoutMillis &&
                    Math.abs(getAverageEncoderPosition(motors) - initialPosition) < countsToTravel) {
                double error = countsToTravel - (getAverageEncoderPosition(motors) - initialPosition);
                errorSum += error;
                double dError = error - lastError;

                double power = ((error * Kp) + (errorSum * Ki) + (dError * Kd));
                power = -power; // Reversed for strafing right

                for (DcMotor motor : motors) {
                    double individualMotorCorrection = calculateIndividualMotorCorrection(motor);
                    // If strafing right, adjust power and apply correction factor
                    if (motor == left_front_motor || motor == right_rear_motor) {
                        setMotorPowerWithRamp(motor, -power * individualMotorCorrection, speed, MOTOR_ACCELERATION);
                    } else {
                        setMotorPowerWithRamp(motor, power * individualMotorCorrection, speed, MOTOR_ACCELERATION);
                    }
                }

                lastError = error;
            }

            stopMotors(motors); // Stop the motors after reaching the target distance or timeout
        }

        /*
        // Move the robot backward a specified distance using PID control and ramped speed.
        public void moveBackwardPID(double targetDistance, double speed) {
            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            StackTraceElement[] stacktrace = Thread.currentThread().getStackTrace();
            StackTraceElement e = stacktrace[3];//maybe this number needs to be corrected
            RobotLog.dd("GAMLOG", "current method: "+name+": called from: "+e);

            // Determine the total encoder distance needed for each motor
            double totalTargetDistance = Math.abs(targetDistance);
            double targetEncoderTicks = totalTargetDistance * ENCODER_COUNTS_PER_INCH;
            DcMotor[] motors = {left_front_motor, right_front_motor, left_rear_motor, right_rear_motor};
            // Initialize PID variables
            double error, integral, derivative, powerLeft, powerRight;
            double previousErrorLeft = 0.0;
            double previousErrorRight = 0.0;
            double integralTermLeft = 0.0;
            double integralTermRight = 0.0;
            double timestamp = runtime.milliseconds();

            // Start moving backwards with initial power
            setMotorPowerWithRamp(left_front_motor, -speed, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
            setMotorPowerWithRamp(right_front_motor, -speed, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
            setMotorPowerWithRamp(left_rear_motor, -speed, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
            setMotorPowerWithRamp(right_rear_motor, -speed, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);

            // Keep moving until both motors reach target distance
            while (Math.abs(getAverageEncoderPosition(motors) - targetEncoderTicks) > ERROR_THRESHOLD ) {
                // Calculate individual errors for each motor
                errorLeft = targetEncoderTicks - getEncoderDistance(left_front_motor);
                errorRight = targetEncoderTicks - getEncoderDistance(right_front_motor);

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
                setMotorPowerWithRamp(left_front_motor, powerLeft, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
                setMotorPowerWithRamp(right_front_motor, powerRight, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
                setMotorPowerWithRamp(left_rear_motor, powerLeft, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
                setMotorPowerWithRamp(right_rear_motor, powerRight, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);

                // Update previous errors and timestamp
                previousErrorLeft = errorLeft;
                previousErrorRight = errorRight;
                timestamp = runtime.milliseconds();
                RobotLog.dd("GAMLOG-DEBUG-"+name,
                        "initial position="+totalTargetDistance
                                +";"+"goal counts to travel="+targetEncoderTicks
                                +";"+"Timeout="+" "
                                +";"+"elapsed time="+String.valueOf(timestamp)
                                +";"+"powerLeft="+(speed*-1)
                                +";"+"errorleft="+errorLeft
                                +";"+"Kp"+Kp
                                +";"+"erroLeft="+errorLeft
                                +";"+"Ki="+Ki
                                +";"+"intergralTermLeft="+integralTermLeft
                                +"Kd"+Kd

                                +"derivativeLeft="+derivativeLeft
                                +";"+"powerRight="+(speed*-1)
                                +";"+"errorRight="+errorRight
                                +";"+"Kp"+Kp
                                +";"+"erroRight="+errorRight
                                +";"+"Ki="+Ki
                                +";"+"intergralTermRight="+integralTermRight
                                +"Kd"+Kd
                                +"derivativeRight="+derivativeRight
                                +";"+"average motor position="+getAverageEncoderPosition(motors)
                                +";"+"motor0 position="+motors[0].getCurrentPosition()
                                +";"+"motor1 position="+motors[1].getCurrentPosition()
                                +";"+"motor2 position="+motors[2].getCurrentPosition()
                                +";"+"motor3 position="+motors[3].getCurrentPosition()
                                +";"+"attempt to set power to left motors= "+powerLeft
                                +";"+"attempt to set power to right motors= "+powerRight
                );
            }

            // Stop motors when target distance is reached
            stopMotors();
        }
*/


        public void turnPID(boolean clockwise, double targetAngle, double speed, long timeoutMillis) {
            DcMotor[] motors = {left_front_motor, right_front_motor, left_rear_motor, right_rear_motor};
            double countsToTurn = targetAngle * ENCODER_COUNTS_PER_DEGREE;

            double initialPosition = getAverageEncoderPosition(motors);

            double errorSum = 0;
            double lastError = 0;
            long startTime = System.currentTimeMillis();

            while ((System.currentTimeMillis() - startTime) < timeoutMillis &&
                    Math.abs(getAverageEncoderPosition(motors) - initialPosition) < countsToTurn) {
                double error = countsToTurn - (getAverageEncoderPosition(motors) - initialPosition);
                errorSum += error;
                double dError = error - lastError;

                double power = ((error * Kp) + (errorSum * Ki) + (dError * Kd));

                // Adjust power distribution for turning clockwise or counterclockwise
                double leftPower = power * (clockwise ? -1.0 : 1.0);
                double rightPower = power * (clockwise ? 1.0 : -1.0);

                double[] powers = {
                        leftPower * calculateIndividualMotorCorrection(left_front_motor),
                        rightPower * calculateIndividualMotorCorrection(right_front_motor),
                        leftPower * calculateIndividualMotorCorrection(left_rear_motor),
                        rightPower * calculateIndividualMotorCorrection(right_rear_motor)
                };

                setMotorPowersWithRamp(motors, powers, speed, MOTOR_ACCELERATION);

                lastError = error;
            }

            stopMotors(motors); // Stop the motors after reaching the target angle or timeout
        }



    // ... Add methods for stopping motors, setting motor mode, etc. ...
        public double getEncoderDistance(DcMotor motor) {
            // Get individual encoder distance for specified motor
            if (motor.equals(left_front_motor)) {
                return left_front_motor.getCurrentPosition();
            } else if (motor.equals(right_front_motor)) {
                return right_front_motor.getCurrentPosition();
            } else if (motor.equals(left_rear_motor)) {
                return left_rear_motor.getCurrentPosition();
            } else if (motor.equals(right_rear_motor)) {
                return right_rear_motor.getCurrentPosition();
            } else {return 0;}
        }
        // Stop all motors
        public void stopMotors() {
            left_front_motor.setPower(0.0);
            right_front_motor.setPower(0.0);
            left_rear_motor.setPower(0.0);
            right_rear_motor.setPower(0.0);
        }

        // Set the motor mode for all motors
        public void setMotorMode(DcMotor.RunMode mode) {
            left_front_motor.setMode(mode);
            right_front_motor.setMode(mode);
            left_rear_motor.setMode(mode);
            right_rear_motor.setMode(mode);
        }

        public double encoderTicksToInches(double encoderTicks) {
            return encoderTicks / ENCODER_COUNTS_PER_INCH;
        }

        public double inchesToEncoderTicks(double distance) {
            return distance * ENCODER_COUNTS_PER_INCH;
        }

        public void resetEncoders() {
            left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Ensure encoders are in RUN_USING_ENCODER mode before resuming operation
            left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public double getAverageEncoderPosition() {
            double leftEncoderPosition = left_front_motor.getCurrentPosition() + left_rear_motor.getCurrentPosition();
            double rightEncoderPosition = right_front_motor.getCurrentPosition() + right_rear_motor.getCurrentPosition();
            return (leftEncoderPosition + rightEncoderPosition) / 2;
        }
        private double getAverageEncoderDistance() {
            // Get individual encoder distances for each motor
            double leftDistance = getEncoderDistance(left_front_motor);
            double rightDistance = getEncoderDistance(right_front_motor);
            double rearLeftDistance = getEncoderDistance(left_rear_motor);
            double rearRightDistance = getEncoderDistance(right_rear_motor);

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
            setMotorPowerWithRamp(left_front_motor, 0.2, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
            setMotorPowerWithRamp(right_front_motor, 0.2, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
            setMotorPowerWithRamp(left_rear_motor, 0.2, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
            setMotorPowerWithRamp(right_rear_motor, 0.2, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);

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
                setMotorPowerWithRamp(left_front_motor, powerLeft, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
                setMotorPowerWithRamp(right_front_motor, powerRight, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
                setMotorPowerWithRamp(left_rear_motor, powerLeft, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);
                setMotorPowerWithRamp(right_rear_motor, powerRight, MAX_MOTOR_SPEED, MOTOR_ACCELERATION);

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

        public void driveRobotDistanceForward(double distanceInCM, double targetSpeed, UnitOfMeasure UOM) {
            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            StackTraceElement[] stacktrace = Thread.currentThread().getStackTrace();
            StackTraceElement e = stacktrace[3];//maybe this number needs to be corrected
            String methodName = e.getMethodName();
            RobotLog.dd("GAMLOG", "current method: "+name+": called from: "+e);
            //RobotLog.dd("GAMLOG", "calling method: "+e);

            int targetCount;

            //convert to number cycles to drive
            //counts_per_rotation/circumference*distane
            if (UOM == UnitOfMeasure.INCHES) {
                targetCount = (int) Math.round(COUNTS_PER_GEAR_REV / (WHEEL_DIAMETER_INCHES * Math.PI) * distanceInCM);
            } else targetCount = (int) Math.round(COUNTS_PER_GEAR_REV / WHEEL_CIRCUMFERENCE * distanceInCM);

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
        /////////////// from driveutil 2023
        public void stopRobot() {
            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            //RobotLog.dd("GAMLOG", "current method: "+name);
            StackTraceElement[] stacktrace = Thread.currentThread().getStackTrace();
            StackTraceElement e = stacktrace[3];//maybe this number needs to be corrected
            String methodName = e.getMethodName();
            RobotLog.dd("GAMLOG", "current method: "+name+": called from: "+e);
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
            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            RobotLog.dd("GAMLOG", "current method: "+name);
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
            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            RobotLog.dd("GAMLOG", "current method: "+name);
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
            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            RobotLog.dd("GAMLOG", "current method: "+name);
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
            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            RobotLog.dd("GAMLOG", "current method: "+name);
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
            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            StackTraceElement[] stacktrace = Thread.currentThread().getStackTrace();
            StackTraceElement e = stacktrace[3];//maybe this number needs to be corrected
            RobotLog.dd("GAMLOG", "current method: "+name+": called from: "+e);

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
            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            RobotLog.dd("GAMLOG", "current method: "+name);
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
            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            RobotLog.dd("GAMLOG", "current method: "+name);
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
            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            RobotLog.dd("GAMLOG", "current method: "+name);
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
            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            RobotLog.dd("GAMLOG", "current method: "+name);
            int targetCount;
            double targetSpeed = 0.5;
            double diameter = 80;//56.8;   //diameter in cms measured between left front and right rear or RF and LR

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
            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            RobotLog.dd("GAMLOG", "current method: "+name);
            int targetCount;
            double targetSpeed = 0.5;
            double diameter = 80;//56.8;   //diameter in cms measured between left front and right rear or RF and LR

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
            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            RobotLog.dd("GAMLOG", "current method: "+name);
            int targetCount;
            double targetSpeed = 0.5;
            double diameter = 80;//56.8;   //diameter in cms measured between left front and right rear or RF and LR

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

        public void rotateLeft45Degrees() {
            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            RobotLog.dd("GAMLOG", "current method: "+name);
            int targetCount;
            double targetSpeed = 0.5;
            double diameter = 80;//56.8;  //diameter in cms measured between left front and right rear or RF and LR

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

        DcMotor reverseMotor(DcMotor thisMotor) {
            /****************
             * Use this to reverse the motor direction so encoding will work
             * @param thisMotor
             * @return reversed motor
             */
            if(thisMotor.getDirection()==DcMotor.Direction.FORWARD) thisMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            else thisMotor.setDirection(DcMotor.Direction.FORWARD);

            return thisMotor;
        }

        public void setMotorPowers(double lf, double lr, double rr, double rf) {
            String name = new Object(){}.getClass().getEnclosingMethod().getName();
            RobotLog.dd("GAMLOG", "current method: "+name);
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
            double leftFrontPower    =  0;
            double rightFrontPower   =  0;
            double leftBackPower     =  0;
            double rightBackPower    =  0;
            double L_HYPOTENUSE;
            double R_HYPOTENUSE;

            L_HYPOTENUSE = Math.sqrt(left_stick_y * left_stick_y +
                    left_stick_x * left_stick_x);
            if (L_HYPOTENUSE == 0) {
                leftFrontPower = 0;
                leftBackPower = 0;
            } else {
                leftFrontPower = -left_stick_y *
                        Math.abs(left_stick_y / L_HYPOTENUSE);
                leftFrontPower += left_stick_x *
                        Math.abs(left_stick_x / L_HYPOTENUSE);
                leftBackPower = -left_stick_y *
                        Math.abs(left_stick_y / L_HYPOTENUSE);
                leftBackPower -= left_stick_x *
                        Math.abs(left_stick_x / L_HYPOTENUSE);
            }
            R_HYPOTENUSE = Math.sqrt(right_stick_y * right_stick_y +
                    right_stick_x * right_stick_x);
            if (R_HYPOTENUSE == 0) {
                rightFrontPower = 0;
                rightBackPower = 0;
            } else {
                rightFrontPower = -right_stick_y *
                        Math.abs(right_stick_y / R_HYPOTENUSE);
                rightFrontPower += right_stick_x *
                        Math.abs(right_stick_x / R_HYPOTENUSE);
                rightBackPower = -right_stick_y *
                        Math.abs(right_stick_y / R_HYPOTENUSE);
                rightBackPower -= right_stick_x *
                        Math.abs(right_stick_x / R_HYPOTENUSE);
            }

            //Ensure Power is between -1 and 1, then factor down by DRIVE_SPEED
            leftFrontPower = Range.clip(leftFrontPower, -1.0, 1.0) * DRIVE_SPEED;
            leftBackPower = Range.clip(leftBackPower, -1.0, 1.0) * DRIVE_SPEED;
            rightFrontPower = Range.clip(rightFrontPower, -1.0, 1.0) * DRIVE_SPEED;
            rightBackPower = Range.clip(rightBackPower, -1.0, 1.0) * DRIVE_SPEED;

            // Send calculated power to wheels
            setMotorPowers(leftFrontPower,leftBackPower,rightBackPower,rightFrontPower);
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

        public void turnToHeading(double maxTurnSpeed, double heading) {
            double  headingError  = 0;
            double  turnSpeed  = 0;
            // Run getSteeringCorrection() once to pre-calculate the current error
            getSteeringCorrection(heading, P_DRIVE_GAIN);

            // keep looping while we are still active, and not on heading.
            while ((Math.abs(headingError) > HEADING_THRESHOLD)) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

                // Clip the speed to the maximum permitted value.
                turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

                // Pivot in place by applying the turning correction
                moveRobot(0,0, turnSpeed);

            }

            // Stop all motion;
            moveRobot(0, 0,0);
        }

        public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
            double  targetHeading = 0;
            double  headingError  = 0;
            targetHeading = desiredHeading;  // Save for telemetry

            // Determine the heading current error
            headingError = targetHeading - getHeading();

            // Normalize the error to be within +/- 180 degrees
            while (headingError > 180)  headingError -= 360;
            while (headingError <= -180) headingError += 360;

            // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
            return Range.clip(headingError * proportionalGain, -1, 1);
        }
    }