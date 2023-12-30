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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.team16751.robot.utilities.ArmUtil;
import org.firstinspires.ftc.team16751.robot.utilities.DriveUtil2023;
import org.firstinspires.ftc.team16751.robot.utilities.ClawUtil;
import org.firstinspires.ftc.team16751.PIDarm;
import org.firstinspires.ftc.team16751.robot.utilities.LauncherUtil;

@TeleOp(name="Driver Control Center Stage", group="Teleop")
public class DriverControl_CenterStage extends LinearOpMode {
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

   // LauncherUtil launcherUtil;
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
       // launcherUtil = new LauncherUtil(hardwareMap);


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
        if(gamepad2.left_trigger>0.5){
            claw.openLeftHand();
        } else{

        }
        if(gamepad2.right_trigger>0.5){
            claw.openRightHand();

        }
    }

    private void doArmControls() {
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
        /*
        if(gamepad2.y && !launcherUtil.getToggleState()) {
            launcherUtil.toggleServo();
        }

        if (gamepad2.a && !aButtonState) {
            aButtonState = true;
            if (!launcherUtil.isLauncherMoving()) {
                if (launcherUtil.getLauncherAngleServoPosition() == launcherUtil.LAUNCHER_ANGLE_DOWN_POSITION) {
                    launcherUtil.raiseLauncher();
                } else {
                    launcherUtil.lowerLauncherDown();
                }
            }
        } else if (!gamepad2.a) {
            aButtonState = false;
        }

         */
    }
    public void dotelemetry() {
        telemetry.addLine("Telemetry");
        telemetry.addData("Shoulder Position: ", armUtil.getshoulderMotorPosition());
        telemetry.addData("Elbow Position ", armUtil.getelbowMotorPosition());
        //telemetry.addData("Wrist Position ", claw.getWristPosition());
        //telemetry.addData("Left Claw Position: ", claw.getLeftClawPosition());
        //telemetry.addData("Right Claw Position: ", claw.getRightClawPosition());
        telemetry.update();
    }
} //end program
