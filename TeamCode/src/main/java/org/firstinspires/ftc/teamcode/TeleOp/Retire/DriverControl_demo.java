/* P3 2021-22 season Driver Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.teamcode.TeleOp.Retire;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot_utilities.ClawUtil;
import org.firstinspires.ftc.teamcode.robot_utilities.DriveUtil2023;
import org.firstinspires.ftc.teamcode.robot_utilities.LiftUtil;
import org.firstinspires.ftc.teamcode.robot_utilities.SensorUtil;
@Disabled
@TeleOp(name="Driver Control Demo", group="Teleop")

public class DriverControl_demo extends LinearOpMode {
    DriveUtil2023 drive = new DriveUtil2023(this);
    ClawUtil claw = new ClawUtil(this);
    LiftUtil lift = new LiftUtil(this);
   // SuzanUtil turret = new SuzanUtil(this);
    SensorUtil sensors = new SensorUtil();

    /** The colorSensor field will contain a reference to our color sensor hardware object */
   // NormalizedColorSensor colorSensor;
    ColorSensor color;
    DistanceSensor distanceSensor;
    RevBlinkinLedDriver lights;

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
        //turret.init(hardwareMap);
        sensors.init(hardwareMap);

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);

        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "sensor_color");
        distanceSensor = (DistanceSensor) hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        //default drive move to 1 (arcade)
        int driveMode = 1;


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /***************************************************************
         * Game Execution Area
         * Runs continous until "STOP" pressed on Driver Station
         ***************************************************************/
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

          /*  if (gamepad1.x) {
                drive.driveRobotStrafeLeft(1.0);
            }
            if (gamepad1.b) {
                drive.driveRobotStrafeRight(1.0);
            }
*/
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

            doIntake();
            //doArmLift();

            if(distanceSensor.getDistance(DistanceUnit.CM) <=4.0) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
            else if (time < 85) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
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

            // Send telemetry messages to explain controls and show robot status
            /*telemetry.addData("color test", sensors.getColorValues().red());
            telemetry.addData("sensor status",sensors.upperLimitIsPressed());
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.addData("distance" , distanceSensor.getDistance(DistanceUnit.CM));

             */
            telemetry.update();
        } //end OpModeIsActive
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
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
        /*if (gamepad1.right_bumper) {
            handOffset += claw.HAND_SPEED;
            handOffset = Range.clip(handOffset, -0.5, 0.5);
            claw.setHandPositions(handOffset);
        }

        else if (gamepad1.left_bumper) {
            handOffset -= claw.HAND_SPEED;
            handOffset = Range.clip(handOffset, -0.5, 0.5);
            claw.setHandPositions(handOffset);
        }
        else {
            //do nothing
        }
*/
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
        if(gamepad2.left_trigger>0.2) lift.decreasePosition(100);
        if(gamepad2.right_trigger>0.2) lift.increasePosition(100);

        //Lowers lift all the way down
        if (gamepad2.dpad_down) lift.raiseToPosition(1,1.0);
        //low junction
        if(gamepad2.dpad_left) lift.raiseToPosition(2,1.0);
        //mid junction
        if(gamepad2.dpad_up) lift.raiseToPosition(3,1.0);
        //high junction
        if(gamepad2.dpad_right) lift.raiseToPosition(4,1.0);
    }

    public boolean isSafetoRotate() {
        int safePosition = 400;
        int liftPosition = lift.getMotorPosition();
        if (liftPosition >= safePosition) return true;
        else return false;
    }



} //end program
