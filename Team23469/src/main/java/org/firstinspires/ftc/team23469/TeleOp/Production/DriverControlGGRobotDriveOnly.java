package org.firstinspires.ftc.team23469.TeleOp.Production;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.team23469.robot.utilities.Production.ClawUtil;
import org.firstinspires.ftc.team23469.robot.utilities.Production.DriveUtil2024;
import org.firstinspires.ftc.team23469.robot.utilities.Production.HangerUtil;
import org.firstinspires.ftc.team23469.robot.utilities.Production.LauncherUtil;
import org.firstinspires.ftc.team23469.robot.utilities.Production.LinearSlidesUtil;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Driver Only Control of GG Robot", group = "TELEOP")
public class DriverControlGGRobotDriveOnly extends LinearOpMode {
        DriveUtil2024 drive = new DriveUtil2024(this);
        Deadline gamePadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);
       // private ClawUtil clawUtil;
        private LinearSlidesUtil linearSlides;
       // private LauncherUtil launcherUtil;
        //private HangerUtil hangerUtil;

        private boolean lastTriggerState = false;
        double DRIVE_SPEED = 1.0;
        double forwardPower; // Forward/backward
        double strafePower; // Left/right strafe
        double turnPower; // Turn
        //default drive move to 1 (arcade)
        int driveMode = 1;
        enum hangerMode {
            STATE_MACHINE,
            MANUAL
        }
        private hangerMode currentHangerMode = hangerMode.STATE_MACHINE;

//        RevBlinkinLedDriver lights, rlights;
        Deadline gamepadRateLimit = new Deadline(250, TimeUnit.MILLISECONDS);
        @Override
        public void runOpMode() throws InterruptedException {
            //update status on driver station
            telemetry.addData("Status", "Initializing...");
            telemetry.update();

            initializeHardware();

            telemetry.addData("Status", "Initializing Complete, waiting for start");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                doDrivetrainControls();
               // doLauncherControls();;
                //doClawControls();
                doSlideControls();
               // doHangerControls();
                addTelemetry();
                //doLights();
            }
           // lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
           // rlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);

        }

    private void initializeHardware() throws InterruptedException {
       /* this method will initialize all the hardware used on the robot
            this includes the drive train and any other subsystems for the robot.
            the subsystems will change from year to year depending on the challenge
            the drivetrain usually stays constant year over year.
        */

      /*
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights"); //initialize lights from hub
        rlights = hardwareMap.get(RevBlinkinLedDriver.class, "rlights"); //initialize lights from hub
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD); //start with a blinking gold while init
        rlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD); //start with a blinking gold while init
       */
        Thread.sleep(250); //give enough time to initialize and set light colors
        drive.init(hardwareMap,telemetry); //initialize the drive subsystem
        //clawUtil = new ClawUtil(hardwareMap); //initialize the claw subsystem
       linearSlides = new LinearSlidesUtil(hardwareMap); //initialize the linear slides subsystem

        //launcherUtil = new LauncherUtil(hardwareMap); //initialize the launcher subsystem
        //hangerUtil = new HangerUtil(hardwareMap); //initialize the hanger subsystem
        //clawUtil.setWristState(true);  //set the initial claw state
        //launcherUtil.launchTriggerClose(); //set the initial launcher state
      //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN); //once done, set lights to green
       // rlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN); //once done, set lights to green

    }
    private void doDrivetrainControls() {
        //Set driver speed as a percentage of full (normally set to full)
        if (gamepad1.right_bumper & gamepad1.left_bumper) {
            DRIVE_SPEED = .25; //set drive speed to 25%
        } else if (gamepad1.left_bumper) {
            DRIVE_SPEED = .50; //set drive speed to 50%
        } else if (gamepad1.right_bumper) {
            DRIVE_SPEED = 1.00; //speed boost to 100%
        } else {
            DRIVE_SPEED = DRIVE_SPEED; //default to 70% drive speed
        }
        forwardPower = -gamepad1.left_stick_y; // Forward/backward
        strafePower = gamepad1.left_stick_x; // Left/right strafe
        turnPower = gamepad1.right_stick_x; // Turn

        //toggle drive mode between arcade and field relative.
        //a timer rate limit is set here so that the robot does not register a single press of
        // the button as multiple presses. the software loops many times faster than human interaction
        if (gamePadRateLimit.hasExpired() && gamepad1.start) {
            if (driveMode == 1) { //if the drive mode is already 1 and users presses start, switch to 2
                driveMode = 2;
            } else if (driveMode == 2) {  //if drive mode is 2 switch to 1
                driveMode = 1;
            }
            gamePadRateLimit.reset(); //reset the rate limit timer
        }

        //reset the gyro sensor (yaw) manually if needed
        if (gamePadRateLimit.hasExpired() && gamepad1.back) {
            drive.resetYaw();
        }

        //test..set buttons to strafe left or right.
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
       // if (gamepad1.start) driveMode = 1;
        //if (gamepad1.back) driveMode = 2;


        //call the respective drive mode
        if (driveMode == 1) {
            fieldDrive();
            telemetry.addData("Drive Mode", "Field");
        }
        else if (driveMode == 2) {
            arcadeDrive();
            telemetry.addData("Drive Mode", "Arcade");
        }
        else {
            fieldDrive();
            telemetry.addData("Drive Mode", "Field");
        }
    }
    public void tankDrive() {
        drive.tankDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, DRIVE_SPEED);
    }
    public void arcadeDrive() {
        drive.arcadeDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, DRIVE_SPEED);
    }
    public void fieldDrive() {
        drive.driveFieldOriented(forwardPower, strafePower, turnPower,gamepad1.right_stick_y, DRIVE_SPEED);
    }
  /*  private void doHangerControls() {
        if (gamepad2.x) {
            hangerUtil.lowerHanger();
        } else if (gamepad2.y) {
            hangerUtil.raiseHanger();
        } else if (gamepad2.b) {
            hangerUtil.hangHanger();
        }

        if (gamePadRateLimit.hasExpired() && gamepad2.back) {
            if (currentHangerMode == hangerMode.STATE_MACHINE) {
                currentHangerMode = hangerMode.MANUAL;
            } else if (currentHangerMode == hangerMode.MANUAL) {
                currentHangerMode = hangerMode.STATE_MACHINE;
                hangerUtil.setCurrentState(HangerUtil.HangerState.IDLE);
            }
            gamePadRateLimit.reset(); //reset the rate limit timer
        }

        if (currentHangerMode == hangerMode.MANUAL) {
            if (Math.abs(gamepad2.left_stick_y) > 0.5) {
                double hangerMotorPower = gamepad2.left_stick_y;
                hangerUtil.moveHangerManually(hangerMotorPower);
            } else hangerUtil.moveHangerManually(0);
        }

        if (gamePadRateLimit.hasExpired() && gamepad2.a) {
            if (hangerUtil.currentServoState == HangerUtil.HangerServoState.LOWER_HANGER_SERVO) { //if the drive mode is already 1 and users presses start, switch to 2
                hangerUtil.raiseHangerServo();
            } else if (hangerUtil.currentServoState == HangerUtil.HangerServoState.RAISE_HANGER_SERVO) {  //if drive mode is 2 switch to 1
                hangerUtil.lowerHangerServo();
            }
            gamePadRateLimit.reset(); //reset the rate limit timer
        }

        telemetry.addData("currentHangerMode: ", currentHangerMode);
        telemetry.addData("requested power: ", -gamepad2.left_stick_y);
    }*/

  private void doSlideControls() {

        if(gamepad2.start) {
            linearSlides.resetEncoder();
        }

        if (linearSlides.isTouchSensorPressed()){
            linearSlides.stopSlides();
            linearSlides.resetEncoder();
        }

        if(gamepad2.right_stick_y==1.0) {
            if(!linearSlides.isTouchSensorPressed()) {
                linearSlides.decreasePosition(100);
            } else if (linearSlides.isTouchSensorPressed()) {
                linearSlides.resetEncoder();
            }
        }

        if(gamepad2.right_stick_y==-1.0) linearSlides.increasePosition(50);

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

   /*
    private void doClawControls() {

        clawUtil.toggleClawWithBumper(gamepad2.left_bumper);
        clawUtil.toggleWristWithBumper(gamepad2.right_bumper);

    }
    */

    /*
    public void doLauncherControls(){

        // Y button rising edge detection and deadline ratelimit
        if (gamepadRateLimit.hasExpired() && gamepad2.left_trigger >0.5) {
            launcherUtil.raiseLauncher();
            gamepadRateLimit.reset();
        }

        // A button rising edge detection and deadline ratelimit
        if (gamepadRateLimit.hasExpired() && gamepad2.right_trigger > 0.5 ) {
            launcherUtil.launchTriggerOpen();
            gamepadRateLimit.reset();
        }
        }
     */

  /*  private void doLights() {
        if (time < 85) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
            rlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
        }
        else if (time >= 85 && time <= 90) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
            rlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
        }
        else if (time > 90 && time < 110) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
            rlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        }
        else if (time >= 110 && time <= 120) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
            rlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
        }
        else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE);
            rlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE);
        }
    }

   */

    private void addTelemetry() {
        // Telemetry to display slide and servo status
        telemetry.addData("left joystick y", gamepad2.left_stick_y);


        // Telemetry for debuggin
        telemetry.addLine("Hanger Telemetry");
        telemetry.addData("Requested Hanger Servo Position", gamepad2.left_stick_x);

        telemetry.update();
    }
}
