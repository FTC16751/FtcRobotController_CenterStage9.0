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
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.team16751.experiments.Datalogger;
import org.firstinspires.ftc.team16751.robot.utilities.ArmUtil_preState;
import org.firstinspires.ftc.team16751.robot.utilities.ClawUtil;
import org.firstinspires.ftc.team16751.robot.utilities.LauncherUtil;
import org.firstinspires.ftc.team16751.robot.utilities.NewDriveUtil2024;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Driver Control Center Stage", group="00-Teleop")
public class DriverControl_CenterStage extends LinearOpMode {
    Datalog datalog;
    Deadline gamepadRateLimit = new Deadline(250, TimeUnit.MILLISECONDS);
    Deadline clawCloseRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);

    NewDriveUtil2024 drive = new NewDriveUtil2024(this);
    ArmUtil_preState armUtil = new ArmUtil_preState(this);
    ClawUtil claw = new ClawUtil(this);
    /** The colorSensor field will contain a reference to our color sensor hardware object */
    // NormalizedColorSensor colorSensor;
    ColorSensor color;
    NormalizedColorSensor distanceLeft;
    NormalizedColorSensor distanceRight;
    RevBlinkinLedDriver lights, rlights;
    int temp = 1;
    double DRIVE_SPEED = 0.9;
    //default drive move to 1 (arcade)
    int driveMode = 1;
    double handOffset   = 0;

    LauncherUtil launcherUtil;
    boolean aButtonState = false;
    VoltageSensor battery;
    TouchSensor leftMag;
    TouchSensor rightMag;
    @Override

    //Initialize and run program
    public void runOpMode() throws InterruptedException {
        hardwareInit();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        datalog.opModeStatus.set("RUNNING");
        /***************************************************************
         * Game Execution Area
         * Runs continous until "STOP" pressed on Driver Station
         ***************************************************************/
        while (opModeIsActive()) {
            for (int i = 0; opModeIsActive(); i++) {
                doDriverControls();
                doArmControls();
                // Handle arm movement based on ArmUtil's state
                handleArmState();
                doClawControls();
                dotelemetry();
                dolauncher();
                setLights();

                // Note that the order in which we set datalog fields
                // does *not* matter! The order is configured inside
                // the Datalog class constructor.

                datalog.loopCounter.set(i);
                datalog.battery.set(battery.getVoltage());
                datalog.FrontLeftMotorPower.set(drive.left_front_motor.getPower());
                datalog.FrontRightMotorPower.set(drive.right_front_motor.getPower());
                datalog.RearLeftMotorPower.set(drive.left_rear_motor.getPower());
                datalog.RearRightMotorPower.set(drive.right_rear_motor.getPower());
                datalog.LeftShoulderMotorPower.set(armUtil.shoulderLeft.getPower());
                datalog.RightShoulderMotorPower.set(armUtil.shoulderRight.getPower());
                datalog.LeftElbowMotorPower.set(armUtil.elbowLeft.getPower());
                datalog.RightElbowMotorPower.set(armUtil.elbowRight.getPower());
                datalog.LeftShoulderMotorCurrent.set(armUtil.shoulderLeft.getCurrent(CurrentUnit.AMPS));
                datalog.RightShoulderMotorCurrent.set(armUtil.shoulderRight.getCurrent(CurrentUnit.AMPS));
                datalog.LeftElbowMotorCurrent.set(armUtil.elbowLeft.getCurrent(CurrentUnit.AMPS));
                datalog.RightElbowMotorCurrent.set(armUtil.elbowRight.getCurrent(CurrentUnit.AMPS));

                // The logged timestamp is taken when writeLine() is called.
                datalog.writeLine();
            }
        } //end OpModeIsActive
    }  //end runOpMode

    private void autoHomeArm() {
        if (!leftMag.isPressed()) {
        while (!leftMag.isPressed()) {
            armUtil.setMotorPower(armUtil.shoulderLeft,-.15);
            armUtil.setMotorPower(armUtil.shoulderRight,-.15);
        }

        armUtil.stopShoulderMotors();
        armUtil.stopAndResetArmMotors();
    }
    }

    private void hardwareInit() {
        battery = hardwareMap.voltageSensor.get("Control Hub");
        // Initialize the datalog
        datalog = new Datalog("datalog_01");

        // You do not need to fill every field of the datalog
        // every time you call writeLine(); those fields will simply
        // contain the last value.
        datalog.opModeStatus.set("INIT");
        datalog.battery.set(battery.getVoltage());
        datalog.writeLine();

        //update status on driver station
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        //init external hardware classes
        drive.init(hardwareMap,telemetry);
        armUtil.init(hardwareMap);
        claw.init(hardwareMap);
        //armUtil.autoHomeArm();
        armUtil.setCurrentState(ArmUtil_preState.ArmState.INIT);
        launcherUtil = new LauncherUtil(hardwareMap);
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        rlights = hardwareMap.get(RevBlinkinLedDriver.class, "rlights");
        rlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);

        leftMag = hardwareMap.get(TouchSensor.class, "lmagswitch");
        rightMag = hardwareMap.get(TouchSensor.class, "rmagswitch");
        distanceLeft = hardwareMap.get(NormalizedColorSensor.class, "lcolorsensor");
        distanceRight = hardwareMap.get(NormalizedColorSensor.class, "rcolorsensor");
    }


    private void doDriverControls() {
        //Set driver speed as a percentage of full (normally set to full)
        if (gamepad1.right_bumper & gamepad1.left_bumper) {
            DRIVE_SPEED = .25;
        } else if (gamepad1.left_bumper) {
            DRIVE_SPEED = .50;
        } else if (gamepad1.right_bumper) {
            DRIVE_SPEED = .75;
        } else {
            DRIVE_SPEED = 1.00;
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
        claw.toggleLeftClawWithBumper(gamepad2.left_bumper);
        claw.toggleRightClawWithBumper(gamepad2.right_bumper);
        /*
        if(gamepad2.left_bumper){
            claw.setClawOpen();
           // claw.openLeftHand();
        }
        if(gamepad2.right_bumper){
            claw.setClawClosed();
            //claw.closeLeftHand();
        }

        if(((DistanceSensor) distanceRight).getDistance(DistanceUnit.CM)<6.0 && clawCloseRateLimit.hasExpired()){
            claw.closeRightHand();
            clawCloseRateLimit.reset();
        }
        if(((DistanceSensor)distanceLeft).getDistance(DistanceUnit.CM)<6.0 && clawCloseRateLimit.hasExpired()){
            claw.closeLeftHand();
            clawCloseRateLimit.reset();

        }

         */
    }

    private void doArmControls() {
        if (gamepad2.back) {
            armUtil.stopElbowMotors();
            armUtil.stopShoulderMotors();
            armUtil.stopAndResetArmMotors();
        }
        if (gamepad2.start) {
            autoHomeArm();
        }
        //Controls for Pre-set arm locations
        //Resting
        if(gamepad2.dpad_left && gamepad2.a && gamepadRateLimit.hasExpired()){
            armUtil.setCurrentState(ArmUtil_preState.ArmState.THIRD_STACK);
            gamepadRateLimit.reset();
        }
        if(gamepad2.dpad_left&&gamepad2.x && gamepadRateLimit.hasExpired()){
            armUtil.setCurrentState(ArmUtil_preState.ArmState.FOURTH_STACK);
            gamepadRateLimit.reset();

        }
        if(gamepad2.dpad_left&&gamepad2.y && gamepadRateLimit.hasExpired()){
            armUtil.setCurrentState(ArmUtil_preState.ArmState.TOP_STACK);
            gamepadRateLimit.reset();

        }
        if(gamepad2.dpad_left&&gamepad2.b && gamepadRateLimit.hasExpired()){
            armUtil.setCurrentState(ArmUtil_preState.ArmState.TOP_STACK);
            gamepadRateLimit.reset();

        }
        if(gamepad2.dpad_down && gamepadRateLimit.hasExpired()){
            claw.setClawClosed();
            armUtil.setCurrentState(ArmUtil_preState.ArmState.INIT);
            gamepadRateLimit.reset();

        }
        if(gamepad2.dpad_left && gamepadRateLimit.hasExpired()){
            armUtil.setCurrentState(ArmUtil_preState.ArmState.LOWER_SCORE);
            gamepadRateLimit.reset();

        }
        //transport
        if(gamepad2.x && gamepadRateLimit.hasExpired()){
            claw.setClawClosed();
            armUtil.setCurrentState(ArmUtil_preState.ArmState.TRANSPORT);
            gamepadRateLimit.reset();

        }
        //LowScore now mid score
        if(gamepad2.dpad_up && gamepadRateLimit.hasExpired()){
            claw.setClawClosed();
            armUtil.setCurrentState(ArmUtil_preState.ArmState.LOW_SCORE);
            gamepadRateLimit.reset();

        }
        //high score
        if(gamepad2.dpad_right && gamepadRateLimit.hasExpired()){
            claw.setClawClosed();
            armUtil.setCurrentState(ArmUtil_preState.ArmState.HIGH_SCORE);
            gamepadRateLimit.reset();

        }
        if(gamepad2.b && !gamepad2.start){
            claw.setClawClosed();

            armUtil.setCurrentState(ArmUtil_preState.ArmState.BACK_LOW_SCORE_DRIVER);
            gamepadRateLimit.reset();

        }
        if(gamepad2.y && gamepadRateLimit.hasExpired()) {
            claw.setClawClosed();
            armUtil.setCurrentState(ArmUtil_preState.ArmState.HANG_READY);
        }

        if(gamepad2.a && gamepadRateLimit.hasExpired()) {
            armUtil.setCurrentState(ArmUtil_preState.ArmState.HANG);
        }

        if (Math.abs(gamepad2.left_stick_y) >0.5 ) {
            if (gamepad2.left_stick_y>0.5) armUtil.decreaseShoulderPosition(50);
            if (gamepad2.left_stick_y<0.5) armUtil.increaseShoulderPosition(50);
        }
        if (Math.abs(gamepad2.right_stick_y) >0.5 ) {
            if (gamepad2.right_stick_y>0.5) armUtil.decreaseElbowPosition(50);
            if (gamepad2.right_stick_y<0.5) armUtil.increaseElbowPosition(50);
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
            rlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);

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
        //from 91seconds to 94 seconds
        //(time > 85 && time <= 120)
        else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            rlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        }
         if(((DistanceSensor) distanceRight).getDistance(DistanceUnit.CM)<6.3){
            rlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
         if(((DistanceSensor)distanceLeft).getDistance(DistanceUnit.CM)<6.3){
             lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
         }
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
        telemetry.addData("Left Shoulder Current: ", armUtil.getLeftShoulderCurrent());
        telemetry.addData("Right Shoulder Current: ", armUtil.getRightShoulderCurrent());
        telemetry.addData("Left Elbow Current: ", armUtil.getLeftElbowCurrent());
        telemetry.addData("Right Elbow Current: ", armUtil.getRightElbowCurrent());
        telemetry.addData("left is pressed",leftMag.isPressed());
        telemetry.addData("right is pressed", rightMag.isPressed());
        telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) distanceLeft).getDistance(DistanceUnit.CM));
        telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) distanceRight).getDistance(DistanceUnit.CM));
        telemetry.addData("RevBlinkin Connection Info: " , lights.getConnectionInfo());
        telemetry.addData("RevBlinkin Connection Info: " , rlights.getConnectionInfo());
        telemetry.addData("gamepad2.dpad_left : ", gamepad2.dpad_left);
        telemetry.addData("gamepad2.x : ", gamepad2.x);
        telemetry.addData("gamepad2.y : ", gamepad2.y);
        telemetry.addData("gamepad2.b : ", gamepad2.b);
        telemetry.addData("gamepad2.a : ", gamepad2.a);
        telemetry.addData("sent wrist position: ", armUtil.getWristPosition());



        telemetry.update();
    }
    /*
     * This class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog
    {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");
        public Datalogger.GenericField loopCounter  = new Datalogger.GenericField("Loop Counter");
        public Datalogger.GenericField battery      = new Datalogger.GenericField("Battery");
        public Datalogger.GenericField FrontLeftMotorPower      = new Datalogger.GenericField("Front Left Motor Power");
        public Datalogger.GenericField FrontRightMotorPower      = new Datalogger.GenericField("Front Right Motor Power");
        public Datalogger.GenericField RearLeftMotorPower      = new Datalogger.GenericField("Rear Left Motor Power");
        public Datalogger.GenericField RearRightMotorPower      = new Datalogger.GenericField("Rear Right Motor Power");

        public Datalogger.GenericField LeftShoulderMotorPower      = new Datalogger.GenericField("Left Shoulder Motor Power");
        public Datalogger.GenericField RightShoulderMotorPower      = new Datalogger.GenericField("Right Shoulder Motor Power");
        public Datalogger.GenericField LeftElbowMotorPower      = new Datalogger.GenericField("Left Elbow Motor Power");
        public Datalogger.GenericField RightElbowMotorPower      = new Datalogger.GenericField("Right Elbow Motor Power");

        public Datalogger.GenericField LeftShoulderMotorCurrent      = new Datalogger.GenericField("Left Shoulder Motor Current");
        public Datalogger.GenericField RightShoulderMotorCurrent      = new Datalogger.GenericField("Right Shoulder Motor Current");
        public Datalogger.GenericField LeftElbowMotorCurrent      = new Datalogger.GenericField("Left Elbow Motor Current");
        public Datalogger.GenericField RightElbowMotorCurrent      = new Datalogger.GenericField("Right Elbow Motor Current");
        public Datalog(String name)
        {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            opModeStatus,
                            loopCounter,
                            battery,
                            FrontLeftMotorPower,
                            FrontRightMotorPower,
                            RearLeftMotorPower,
                            RearRightMotorPower,
                            LeftShoulderMotorPower,
                            RightShoulderMotorPower,
                            LeftElbowMotorPower,
                            RightElbowMotorPower,
                            LeftShoulderMotorCurrent,
                            RightShoulderMotorCurrent,
                            LeftElbowMotorCurrent,
                            RightElbowMotorCurrent
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }
} //end program