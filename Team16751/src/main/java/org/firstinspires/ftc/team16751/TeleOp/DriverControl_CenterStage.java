/* P3 2021-22 season Driver Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.team16751.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.team16751.robot.utilities.ArmUtil;
import org.firstinspires.ftc.team16751.robot.utilities.DriveUtil2023;
import org.firstinspires.ftc.team16751.robot.utilities.ClawUtil;
import org.firstinspires.ftc.team16751.robot.utilities.LauncherUtil;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Driver Control Center Stage", group="Teleop")
public class DriverControl_CenterStage extends LinearOpMode {
    Deadline gamepadRateLimit = new Deadline(250, TimeUnit.MILLISECONDS);

    DriveUtil2023 drive = new DriveUtil2023(this);
    ArmUtil armUtil = new ArmUtil(this);
    ClawUtil claw = new ClawUtil(this);
    /** The colorSensor field will contain a reference to our color sensor hardware object */
    // NormalizedColorSensor colorSensor;
    ColorSensor color;
    DistanceSensor distanceSensor;
    RevBlinkinLedDriver lights;
    int temp = 1;
    double DRIVE_SPEED = 1;
    //default drive move to 1 (arcade)
    int driveMode = 1;
    double handOffset   = 0;

    LauncherUtil launcherUtil;
    boolean aButtonState = false;

    @Override

    //Initialize and run program
    public void runOpMode() throws InterruptedException {
        //update status on driver station
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        //init external hardware classes
        drive.init(hardwareMap);
        armUtil.init(hardwareMap);
        claw.init(hardwareMap);
        armUtil.setCurrentState(ArmUtil.ArmState.INIT);
        launcherUtil = new LauncherUtil(hardwareMap);

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /***************************************************************
         * Game Execution Area
         * Runs continous until "STOP" pressed on Driver Station
         ***************************************************************/
        while (opModeIsActive()) {

            doDriverControls();
            doArmControls();
            // Handle arm movement based on ArmUtil's state
            handleArmState();
            doClawControls();
            dotelemetry();
            dolauncher();
            setLights();

        } //end OpModeIsActive
    }  //end runOpMode



    private void doDriverControls() {
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
        /***end of set drive mode code */
    }

    private void doClawControls() {
        if(gamepad2.left_bumper){
            claw.setClawOpen();
        }
        if(gamepad2.right_bumper){
            claw.setClawClosed();
        }


    }

    private void doArmControls() {
        if (gamepad2.back) {
            armUtil.stopElbowMotors();
            armUtil.stopShoulderMotors();
            armUtil.stopAndResetArmMotors();
        }
        //Controls for Pre-set arm locations
        //Resting
        if(gamepad2.dpad_down){
            armUtil.setCurrentState(ArmUtil.ArmState.INIT);
        }
        //transport
        if(gamepad2.dpad_left){
            armUtil.setCurrentState(ArmUtil.ArmState.TRANSPORT);
        }
        //LowScore
        if(gamepad2.dpad_up){
            armUtil.setCurrentState(ArmUtil.ArmState.LOW_SCORE);
        }
        //high score
        if(gamepad2.dpad_right){
            armUtil.setCurrentState(ArmUtil.ArmState.HIGH_SCORE);
        }
        if(gamepad2.b){
            armUtil.setCurrentState(ArmUtil.ArmState.BACK_LOW_SCORE_DRIVER);
        }
        if(gamepad2.y) {
            armUtil.setCurrentState(ArmUtil.ArmState.HANG_READY);
        }
        if(gamepad2.a) {
            armUtil.setCurrentState(ArmUtil.ArmState.HANG);
        }
        if (Math.abs(gamepad2.left_stick_y) >0.5 ) {
            armUtil.decreaseShoulderPosition(100);
        }


        if (gamepadRateLimit.hasExpired() && gamepad2.left_trigger > 0.5 ) {
            armUtil.decrementWristPosition();
            gamepadRateLimit.reset();
        }

        if (gamepadRateLimit.hasExpired() && gamepad2.right_trigger > 0.5 ) {
            armUtil.incrementWristPosition();
            gamepadRateLimit.reset();
        }


    }
    private void handleArmState() {
        armUtil.runStateMachine();
    }

    public void tankDrive() {
        drive.tankDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, DRIVE_SPEED);
    }

    public void arcadeDrive() {
        drive.arcadeDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, DRIVE_SPEED);
    }

    public void dolauncher()
    {

        // Y button rising edge detection and deadline ratelimit
        if (gamepadRateLimit.hasExpired() && gamepad2.left_stick_button) {
            launcherUtil.toggleLaunchAngleServo();
            gamepadRateLimit.reset();
        }

        // A button rising edge detection and deadline ratelimit
        if (gamepadRateLimit.hasExpired() && gamepad2.right_stick_button) {
            launcherUtil.toggleServo();
            gamepadRateLimit.reset();
        }

    }
    public void setLights(){
        if (time < 85) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        }
        else if (time >= 85 && time <= 90) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
        }
        else if (time > 90 && time < 110) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        }
        else if (time >= 110 && time <= 120) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
        }
        //from 91seconds to 94 seconds
        //(time > 85 && time <= 120)
        else
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
    } //end OpModeIsActive
    public void dotelemetry() {
        telemetry.addLine("Telemetry");
        telemetry.addData("Left Shoulder Position: ", armUtil.getLeftShoulderMotorPosition());
        telemetry.addData("Right Shoulder Position: ", armUtil.getRightShoulderMotorPosition());
        telemetry.addData("Left Elbow Position ", armUtil.getLeftElbowMotorPosition());
        telemetry.addData("Right Elbow Position ", armUtil.getRightElbowMotorPosition());
        telemetry.addData("right stick y: ", gamepad2.right_stick_y);
        telemetry.addData("current state: ", armUtil.getCurrentState());
        telemetry.addData("robot heading: ", drive.getHeading());
       // telemetry.addData("Wrist Position ", claw.getWristPosition());
        //telemetry.addData("Left Claw Position: ", claw.getLeftClawPosition());
        //telemetry.addData("Right Claw Position: ", claw.getRightClawPosition());
        telemetry.update();
    }
} //end program