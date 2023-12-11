/* P3 2021-22 season Driver Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.team23469.TeleOp.Learning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.team23469.robot.utilities.Learning.ClawUtil;
import org.firstinspires.ftc.team23469.robot.utilities.Production.DriveUtil;
import org.firstinspires.ftc.team23469.robot.utilities.Learning.HangerUtil;
import org.firstinspires.ftc.team23469.robot.utilities.Learning.LiftUtil;
import org.firstinspires.ftc.team23469.robot.utilities.Learning.launcherUtil;


@TeleOp(name="Driver Control Center Stage Test", group="Teleop")
@Disabled
    public class DriverControl_CenterStageTEST<wriststate> extends LinearOpMode {
    DriveUtil drive = new DriveUtil(this);
    launcherUtil launcher = new launcherUtil(this);
    HangerUtil hanger = new HangerUtil(this);
    ClawUtil claw = new ClawUtil(this);
    LiftUtil lift = new LiftUtil(this);
    boolean intakeToggle = false;
    boolean wristToggle = false;
    int temp = 1;
    double DRIVE_SPEED = 1;
    double handOffset   = 0;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    enum ClawState {
        OPEN,
        CLOSE
    }

    ClawState clawstate = ClawState.OPEN;
    ClawUtil.WristState wriststate = ClawUtil.WristState.DOWN;

    //subclass is replacing inherited behavior.
    @Override

    //Initialize and run program
    public void runOpMode() throws InterruptedException {
        //update status on driver station
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        //init external hardware classes
        drive.init(hardwareMap);
        launcher.init();
        claw.init();
        hanger.init(hardwareMap);
        lift.init(hardwareMap);

        //default drive move to 1 (arcade)
        int driveMode = 1;


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /***************************************************************
         * Game Execution Area
         * Runs continous until "STOP" pressed on Driver Station
         ***************************************************************/
        while (opModeIsActive()) {
            // Store the gamepad values from the previous loop iteration in
            // previousGamepad1/2 to be used in this loop iteration.
            // This is equivalent to doing this at the end of the previous
            // loop iteration, as it will run in the same order except for
            // the first/last iteration of the loop.
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Store the gamepad values from this loop iteration in
            // currentGamepad1/2 to be used for the entirety of this loop iteration.
            // This prevents the gamepad values from changing between being
            // used and stored in previousGamepad1/2.
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

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
            //doLauncher();
            doClaw();
            doLift();
            //doHanger();
            //toggleWrist();


        } //end OpModeIsActive
    }  //end runOpMode

    public void tankDrive() {
        drive.tankDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, DRIVE_SPEED);
    }

    public void arcadeDrive() {
        drive.arcadeDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, DRIVE_SPEED);
    }
    public void doLauncher() {
        if (gamepad2.left_bumper) {
            launcher.setLauncherUp();

        } else {// continue looping//
        }

        if (gamepad2.right_bumper) {
            launcher.setLauncherDown();
        } else {// continue looping
        }
    }

    public void doHanger() {
        if (gamepad2.x)hanger.raiseToPosition(2, 0.25);
        if (gamepad2.y)hanger.raiseToPosition(1, 0.25);
        if (gamepad2.b)hanger.raiseToPosition(0, 0.25);
    }

    public void doClaw() {
        // Rising edge detector
        if (currentGamepad1.a && !previousGamepad1.a) {
            intakeToggle = !intakeToggle;
        }
        if (intakeToggle) {
            claw.setClawClosed();
        }
        else {
            claw.setClawOpen();
        }

/*
        if (gamepad2.left_bumper){
            if (clawstate == ClawState.OPEN) {
                claw.setClawClosed();
                clawstate = ClawState.CLOSE;
            }
            else if (clawstate == ClawState.CLOSE) {
                claw.setClawOpen();
                clawstate = ClawState.OPEN;
            }
            else {}
        }
*/
    }
    public void doLift() {
        //--------------------------------------------------------------------------
        // code for the arm motor
        // Sets arm to set levels depending on button push (requires encoders)
        //--------------------------------------------------------------------------
        double armPower;

        //Lowers lift all the way down
        if (gamepad2.dpad_down) {
            lift.raiseToPosition(0,1.0);
            claw.setWristGrab();
        }

        //low score
        if(gamepad2.dpad_left) {
            lift.raiseToPosition(1,1.0);
            claw.setWristScore();
        }
        //mid score
        if(gamepad2.dpad_up) lift.raiseToPosition(2,0.25);
        //high core
        if(gamepad2.dpad_right) lift.raiseToPosition(3,0.25);
    }

    public void toggleWrist() {
        // Rising edge detector
        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            wristToggle = !wristToggle;
        }
        if (wristToggle) {
            switch (wriststate) {
                case DOWN:
                    claw.toggleWrist(ClawUtil.WristState.DOWN);
                    if (gamepad2.right_bumper) {
                        wriststate = ClawUtil.WristState.GRAB;
                    } else {
                    }
                    break;
                case GRAB:
                    claw.toggleWrist(ClawUtil.WristState.GRAB);
                    if (gamepad2.right_bumper) {
                        wriststate = ClawUtil.WristState.CARRY;
                    } else {
                    }
                    break;
                case CARRY:
                    claw.toggleWrist(ClawUtil.WristState.CARRY);
                    if (gamepad2.right_bumper) {
                        wriststate = ClawUtil.WristState.SCORE;
                    } else {
                    }
                    break;
                case SCORE:
                    claw.toggleWrist(ClawUtil.WristState.SCORE);
                    if (gamepad2.right_bumper) {
                        wriststate = ClawUtil.WristState.DOWN;
                    } else {
                    }
                    break;
                default:
                    telemetry.addData("State", "Default");
                    break;
            }
        }
        else {
            telemetry.addData("State", "Default");
        }
    }
}