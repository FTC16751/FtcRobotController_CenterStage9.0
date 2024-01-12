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

import org.firstinspires.ftc.team16751.robot.utilities.ArmUtil;
import org.firstinspires.ftc.team16751.robot.utilities.ClawUtil;
import org.firstinspires.ftc.team16751.robot.utilities.DriveUtil2023;

@TeleOp(name="TestDriver", group="Teleop")
public class TestDriver extends LinearOpMode {
    DriveUtil2023 drive = new DriveUtil2023(this);
    ArmUtil armUtil = new ArmUtil(this);
    ClawUtil claw = new ClawUtil(this);
    /** The colorSensor field will contain a reference to our color sensor hardware object */
    // NormalizedColorSensor colorSensor;
    ColorSensor color;
    DistanceSensor distanceSensor;
    RevBlinkinLedDriver lights;
    int temp = 1;
    double DRIVE_SPEED = .75;
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
            arcadeDrive();
    }

    private void doClawControls() {
        if(gamepad1.left_bumper){
            claw.setClawOpen();
        }
        if(gamepad1.right_bumper){
            claw.setClawClosed();
        }

    }

    private void doArmControls() {
        //Controls for Pre-set arm locations
        //Resting
        if(gamepad1.dpad_down){
            armUtil.setCurrentState(ArmUtil.ArmState.INIT);
        }
        //transport
        if(gamepad1.dpad_left){
            armUtil.setCurrentState(ArmUtil.ArmState.TRANSPORT);
        }
        //LowScore
        if(gamepad1.dpad_up){
            armUtil.setCurrentState(ArmUtil.ArmState.LOW_SCORE);
        }
        //high score
        if(gamepad1.dpad_right){
            armUtil.setCurrentState(ArmUtil.ArmState.HIGH_SCORE);
        }
        if(gamepad1.b){
            armUtil.setCurrentState(ArmUtil.ArmState.BACK_LOW_SCORE);
        }
        if(gamepad1.a) {
            armUtil.setCurrentState(ArmUtil.ArmState.HANG_READY);
        }
        if(gamepad1.y) {
            armUtil.setCurrentState(ArmUtil.ArmState.HANG);
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
        telemetry.addData("Left Shoulder Position: ", armUtil.getLeftShoulderMotorPosition());
        telemetry.addData("Right Shoulder Position: ", armUtil.getRightShoulderMotorPosition());
        telemetry.addData("Left Elbow Position ", armUtil.getLeftElbowMotorPosition());
        telemetry.addData("Right Elbow Position ", armUtil.getRightElbowMotorPosition());
       // telemetry.addData("Wrist Position ", claw.getWristPosition());
        //telemetry.addData("Left Claw Position: ", claw.getLeftClawPosition());
        //telemetry.addData("Right Claw Position: ", claw.getRightClawPosition());
        telemetry.update();
    }
} //end program
