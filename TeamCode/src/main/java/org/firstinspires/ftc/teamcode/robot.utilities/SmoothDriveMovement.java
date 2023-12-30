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

public class SmoothDriveMovement {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx rearLeft;
    private DcMotorEx rearRight;
    private double rampRate = 1.00;
    private double lastForwardReverse = 0;
    private double lastStrafe = 0;
    private double lastTurn = 0;
    private double lastUpdateTime = 0; // Declare last update time
    private Telemetry telemetry;
    private IMU imu;
    double firstangle;

    public SmoothDriveMovement(HardwareMap hardwareMap, Telemetry telemetry ) {
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

   public void storeFirstAngle() {
       firstangle = getIMUHeading();
   }

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
        telemetry.addData("Front Left Power", frontLeft.getPower());
        telemetry.addData("Front Right Power", frontRight.getPower());
        telemetry.addData("Rear Left Power", rearLeft.getPower());
        telemetry.addData("Rear Right Power", rearRight.getPower());
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

    public void fielddrive(double leftStickX, double leftStickY, double rightStickX, double rightStickY) {
        double forwardReverse = leftStickY;
        double strafe = leftStickX;
        double turn = rightStickX;

        // Get the current orientation
        Orientation currentOrientation =  imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentyawAngle = currentOrientation.firstAngle;
        //double lastAngle = myRobotOrientation.firstAngle;
        // Calculate the orientation difference since the last reset
        double orientationDifference = currentyawAngle - firstangle;

        // Adjust strafing and forward/reverse motions based on the orientation difference
        double newStrafe = strafe * Math.cos(orientationDifference) - forwardReverse * Math.sin(orientationDifference);
        double newForwardReverse = strafe * Math.sin(orientationDifference) + forwardReverse * Math.cos(orientationDifference);

        double currentTime = System.currentTimeMillis() / 1000.0; // Get time in seconds

        if (Math.abs(leftStickX) < 0.05 && Math.abs(leftStickY) < 0.05 && Math.abs(rightStickX) < 0.05 && Math.abs(rightStickY) < 0.05) {
            // If no joystick input, gradually reduce power to 0
            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);
        } else {
            // Calculate target powers
            double targetFLPower = newForwardReverse + newStrafe + turn;
            double targetFRPower = newForwardReverse - newStrafe - turn;
            double targetRLPower = newForwardReverse - newStrafe + turn;
            double targetRRPower = newForwardReverse + newStrafe - turn;

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
        telemetry.addData("Front Left Power", frontLeft.getPower());
        telemetry.addData("Front Right Power", frontRight.getPower());
        telemetry.addData("Rear Left Power", rearLeft.getPower());
        telemetry.addData("Rear Right Power", rearRight.getPower());
        telemetry.addData("currentyawAngle: ", currentyawAngle);
        telemetry.addData("orientationDifference: ", orientationDifference);
        telemetry.addData("myRobotOrientation.firstAngle: ", myRobotOrientation.firstAngle);
        telemetry.addData("Math.cos(orientationDifference): ", Math.cos(orientationDifference));
        telemetry.addData("Math.sin(orientationDifference): ", Math.sin(orientationDifference));
        telemetry.addData("newstrafe: ", newStrafe);
        telemetry.addData("newForwardReverse: ",newForwardReverse);
        telemetry.addData("forwardReverse: ", leftStickY);
        telemetry.addData("strafe: ", strafe);
        telemetry.addData("turn:", turn);

    }
    public void resetOrientation() {
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    private double getIMUHeading() {
        // Get the orientation from the IMU
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Extract the yaw value for heading
        double yawAngle = myRobotOrientation.firstAngle;

        // Adjust yaw angle to be between 0 and 360 degrees
       // if (yawAngle < 0) {
          //  yawAngle += 360;
        //}

        return yawAngle;
    }


}
