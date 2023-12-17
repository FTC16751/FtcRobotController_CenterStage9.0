package org.firstinspires.ftc.team23469.TeleOp.Production;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team23469.robot.utilities.Production.DriveUtil;
import org.firstinspires.ftc.team23469.robot.utilities.Production.HangerUtil;
import org.firstinspires.ftc.team23469.robot.utilities.Production.LauncherUtil;
import org.firstinspires.ftc.team23469.robot.utilities.Production.LinearSlidesUtil;
import org.firstinspires.ftc.team23469.robot.utilities.Production.ClawUtil;

@TeleOp(name = "Driver Control of GG Robot", group = "coach")
public class DriverControlGGRobot extends LinearOpMode {
        DriveUtil drive = new DriveUtil(this);
        private ClawUtil clawUtil;
        private LinearSlidesUtil linearSlides;
        private LauncherUtil launcherUtil;
        private HangerUtil hangerUtil;
        private boolean lastTriggerState = false;
        double DRIVE_SPEED = 1;

        //default drive move to 1 (arcade)
        int driveMode = 1;
        RevBlinkinLedDriver lights;

        @Override
        public void runOpMode() throws InterruptedException {
            //update status on driver station
            telemetry.addData("Status", "Initializing...");
            telemetry.update();

            initHardware();

            telemetry.addData("Status", "Initializing Complete, waiting for start");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                doDrive();
                doLauncher();;
                doClaw();
                doSlides();
                doHanger();
                addTelemetry();
                doLights();
            }
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
        }

    private void initHardware() throws InterruptedException {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        Thread.sleep(2000);
        drive.init(hardwareMap);
        clawUtil = new ClawUtil(hardwareMap);
        linearSlides = new LinearSlidesUtil(hardwareMap);
        launcherUtil = new LauncherUtil(hardwareMap);
        hangerUtil = new HangerUtil(hardwareMap);
        clawUtil.setWristState(true);
        launcherUtil.setLauncherDown();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    private void doDrive() {
        //Set driver speed as a percentage of full (normally set to full)
        if (gamepad1.right_bumper & gamepad1.left_bumper) {
            DRIVE_SPEED = .25;
        } else if (gamepad1.left_bumper) {
            DRIVE_SPEED = .50;
        } else if (gamepad1.right_bumper) {
            DRIVE_SPEED = 1.00;
        } else {
            DRIVE_SPEED = .80;
        }

        if (gamepad1.x) {
            drive.driveRobotStrafeLeft(1.0);
        }
        if (gamepad1.b) {
            drive.driveRobotStrafeRight(1.0);
        }

        /***************************************************************
         * Set Drive Mode
         * This section of code will allow the driver to set
         * the drive mode between arcade drive and tank drive
         * arcade drive is set when clicking the start button on the controller
         * tank drive is set when clicking the back button on the controller
         * this can be done while in teleop operation
         * default drive mode is arcade drive
         ***************************************************************/
        if (gamepad1.start) driveMode = 1;
        if (gamepad1.back) driveMode = 2;

        //call the respective drive mode
        if (driveMode == 1) {
            arcadeDrive();
            telemetry.addData("Drive Mode", "Arcade");
        }
        else if (driveMode == 2) {
            tankDrive();
            telemetry.addData("Drive Mode", "Tank");
        }
        else {
            arcadeDrive();
            telemetry.addData("Drive Mode", "Arcade");
        }
    }

    public void tankDrive() {
        drive.tankDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, DRIVE_SPEED);
    }

    public void arcadeDrive() {
        drive.arcadeDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, DRIVE_SPEED);
    }

    private void doHanger() {
        boolean currentTriggerState = gamepad2.left_trigger > 0.5;

        if (currentTriggerState && !lastTriggerState) {
            hangerUtil.toggleHanger();
        }

        lastTriggerState = currentTriggerState;

        // Manual overrides using left joystick of gamepad2
        double hangerMotorPower = -gamepad2.left_stick_y;
       // double hangerServoPosition = gamepad2.left_stick_x;

        if (Math.abs(hangerMotorPower) >0.5 ) {
            hangerUtil.setManualMove();
            hangerUtil.moveHangerManually(hangerMotorPower);
        } else {
            hangerUtil.stopMotor();
        }
    }

    private void doSlides() {
        if (gamepad2.dpad_down) {
            linearSlides.setCurrentState(LinearSlidesUtil.SlideState.LEVEL_ZERO);
        } else if (gamepad2.dpad_left) {
            linearSlides.setCurrentState(LinearSlidesUtil.SlideState.LOW_POSITION);
        } else if (gamepad2.dpad_up) {
            linearSlides.setCurrentState(LinearSlidesUtil.SlideState.MID_POSITION);
        } else if (gamepad2.dpad_right) {
            linearSlides.setCurrentState(LinearSlidesUtil.SlideState.HIGH_POSITION);
        }
        linearSlides.runStateMachine();
    }

    private void doClaw() {
        clawUtil.toggleClawWithBumper(gamepad2.left_bumper);
        clawUtil.toggleWristWithBumper(gamepad2.right_bumper);
    }

    public void doLauncher(){
            if (gamepad2.right_trigger > 0.5 && !launcherUtil.isLauncherMoving()) {
                launcherUtil.raiseLauncher();
                while (launcherUtil.isLauncherMoving()) {
                }
                sleep(250);
                launcherUtil.setLauncherUp();
            }
        }

    private void doLights() {
        if (time < 85) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
        }
        else if (time >= 85 && time <= 90) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
        }
        else if (time > 90 && time < 110) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        }
        else if (time >= 115 && time <= 120) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
        }
        else
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);

    }
    private void addTelemetry() {
        // Telemetry to display slide and servo status
        telemetry.addData("Slide State", linearSlides.getCurrentState());
        telemetry.addData("Claw Position", clawUtil.getClawPosition());
        telemetry.addData("Wrist Position", clawUtil.getWristPosition());

        // Telemetry for debuggin
        telemetry.addLine("Hanger Telemetry");
        telemetry.addData("Hanger State", hangerUtil.getCurrentState());
        telemetry.addData("Hanger Motor Position", hangerUtil.getHangerPosition());
        telemetry.addData("Hanger Servo Position", hangerUtil.getHangerServoPosition());
        telemetry.addData("Requested Hanger Servo Position", gamepad2.left_stick_x);
        telemetry.addData("left lift position", linearSlides.getLeftLiftPosition());
        telemetry.addData("right lift position", linearSlides.getRightLiftPosition());

        telemetry.update();
    }
}
