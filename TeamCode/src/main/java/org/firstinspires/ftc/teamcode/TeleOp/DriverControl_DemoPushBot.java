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

@TeleOp(name="Driver Control Push Bot", group="00-Teleop")
public class DriverControl_DemoPushBot extends LinearOpMode {
    NewDriveUtil2024 drive = new NewDriveUtil2024(this);
    Deadline gamePadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);
    int temp = 1;
    double DRIVE_SPEED = .5;
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
        drive.resetYaw();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /** Runs continuous until "STOP" pressed on Driver Station **/
        while (opModeIsActive()) {
            doDriverControls();
            doTelemetry();
        } //end OpModeIsActive
    }  //end runOpMode


    private void doDriverControls() {
        //Set driver speed as a percentage of full (normally set to full)
        //if the driver presses both the left bumper and right  bumper set the speed to slow 25%
        if(gamepad1.right_bumper && gamepad1.left_bumper){
            DRIVE_SPEED = .25;
        }
        //else if the driver presses the left bumper slow to 50%
        else if (gamepad1.left_bumper) {
            DRIVE_SPEED=.5;
        }
        //else if the driver presses the right bumper speed burst 100%
        else if (gamepad1.right_bumper) {
            DRIVE_SPEED=1.0;
        }
        //else set to the default speed
        else {
            DRIVE_SPEED=DRIVE_SPEED;
        }
        forwardPower = -gamepad1.left_stick_y; // Forward/backward
        strafePower = gamepad1.left_stick_x; // Left/right strafe
        turnPower = gamepad1.right_stick_x; // Turn

        //change drive modes from arcade to field
        //if the user presses the gamepad1 start button, the program should switch between
        //arcade mode (drivemode1) and field relative mode (drivemode2).
        //use a gamepad rate limiter to ensure consistent button presses

        if (gamePadRateLimit.hasExpired() && gamepad1.start) {
            if (driveMode == 1) {
                driveMode = 2;
            } else if (driveMode == 2) {
                driveMode = 1;
            } else driveMode = 1;
            //reset the gamepad rate limit
            gamePadRateLimit.reset();
        }



        //call the respective drive mode
        if (driveMode == 1) {
            tankDrive();
            telemetry.addData("Drive Mode", "Tank");
        }
        else if (driveMode == 2) {
            arcadeDrive();
            telemetry.addData("Drive Mode", "Arcade");
        }
        else {
            tankDrive();
            telemetry.addData("Drive Mode", "Tank");
        }

        //if the user presses the b button, rotate teh robot to the right


        if (gamePadRateLimit.hasExpired() && gamepad1.back) {
            drive.resetYaw();
        }
    }


    public void tankDrive() {
        //drive.tankDrive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.right_stick_y, DRIVE_SPEED);
        drive.simpleTankDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, DRIVE_SPEED);

    }

    public void arcadeDrive() {
        //drive.tankDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, DRIVE_SPEED);
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
