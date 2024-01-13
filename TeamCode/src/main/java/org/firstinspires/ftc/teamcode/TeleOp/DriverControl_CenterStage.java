/* P3 2021-22 season Driver Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.robot_utilities.NewDriveUtil2024;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Driver Control Center Stage", group="Teleop")
public class DriverControl_CenterStage extends LinearOpMode {
    NewDriveUtil2024 drive = new NewDriveUtil2024(this);
    Deadline gamePadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);
    int temp = 1;
    double DRIVE_SPEED = 1;
    double forwardPower; // Forward/backward
    double strafePower; // Left/right strafe
    double turnPower; // Turn
    //default drive move to 1 (arcade)
    int driveMode = 1;
    double handOffset   = 0;
    //subclass is replacing inherited behavior.
    @Override

    //Initialize and run program
    public void runOpMode() throws InterruptedException {
        //update status on driver station
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        //init external hardware classes
        drive.init(hardwareMap,telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /** Runs continous until "STOP" pressed on Driver Station **/
        while (opModeIsActive()) {
            doDriverControls();
            doTelemetry();
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
            DRIVE_SPEED = .75;
        }

        forwardPower = -gamepad1.left_stick_y; // Forward/backward
        strafePower = gamepad1.left_stick_x; // Left/right strafe
        turnPower = gamepad1.right_stick_x; // Turn

        if (gamePadRateLimit.hasExpired() && gamepad1.start) {
            if (driveMode == 1) {
                driveMode = 2;
            } else if (driveMode == 2) {
                driveMode = 1;
            }
            gamePadRateLimit.reset();
        }

        if (gamePadRateLimit.hasExpired() && gamepad1.back) {
            drive.resetYaw();
        }

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

        if (gamepad1.x) {
            drive.driveRobotStrafeLeft(1.0);
        }
        if (gamepad1.b) {
            drive.driveRobotStrafeRight(1.0);
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
        //drive.driveFieldOriented(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, DRIVE_SPEED);
    }
    private void doTelemetry() {
        telemetry.update();
    }

} //end program
