/* P3 2021-22 season Driver Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.team23469.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team23469.robot.utilities.ClawUtil;
import org.firstinspires.ftc.team23469.robot.utilities.DriveUtil2023;
import org.firstinspires.ftc.team23469.robot.utilities.LiftUtil;


@TeleOp(name="Driver Control LiftTest", group="Teleop")

public class DriverControl_LiftTest extends LinearOpMode {
    DriveUtil2023 drive = new DriveUtil2023(this);
    ClawUtil claw = new ClawUtil(this);
    LiftUtil lift = new LiftUtil(this);

    int temp = 1;
    double DRIVE_SPEED = 1;
    double handOffset   = 0;
    //subclass is replacing inherited behavior.
    @Override

    //Initialize and run program
    public void runOpMode() throws InterruptedException {
        //update status on driver station
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        //init external hardware classes
        drive.init(hardwareMap);
        claw.init();
        lift.init(hardwareMap);

        //default drive move to 1 (arcade)
        int driveMode = 1;


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            //Set driver speed as a percentage of full (normally set to full)
            if (gamepad1.right_bumper & gamepad1.left_bumper) {
                DRIVE_SPEED = .25;
            } else if (gamepad1.left_bumper) {
                DRIVE_SPEED = .30;
            } else if (gamepad1.right_bumper) {
                DRIVE_SPEED = 1.00;
            } else {
                DRIVE_SPEED = .50;
            }

            if (gamepad2.start) {
                lift.resetEncoder();
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

           // doIntake();
            doArmLift();

            telemetry.update();
        } //end OpModeIsActive
    }  //end runOpMode

    public void tankDrive() {
        drive.tankDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, DRIVE_SPEED);
    }

    public void arcadeDrive() {
        drive.arcadeDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, DRIVE_SPEED);
    }

    public void doIntake() {
        // Use gamepad left & right Bumpers to open and close the claw
        // Use the SERVO constants defined in RobotHardware class.
        // Each time around the loop, the servos will move by a small amount.
        // Limit the total offset to half of the full travel range

        if (gamepad2.left_bumper){
            claw.setClawOpen();
        } else {// continue looping//
        }

        if (gamepad2.right_bumper){
            claw.setClawClosed();
        }else {// continue looping
        }
    }

    public void doArmLift() {
        //--------------------------------------------------------------------------
        // code for the arm motor
        // Sets arm to set levels depending on button push (requires encoders)
        //--------------------------------------------------------------------------
        double armPower;

        //Lowers lift all the way down
        if (gamepad2.dpad_down) lift.raiseToPosition(1,0.25);
        //low junction
        if(gamepad2.dpad_left) lift.raiseToPosition(2,0.25);
        //mid junction
        if(gamepad2.dpad_up) lift.raiseToPosition(3,0.25);
        //high junction
        if(gamepad2.dpad_right) lift.raiseToPosition(0,0.25);
    }
} //end program
