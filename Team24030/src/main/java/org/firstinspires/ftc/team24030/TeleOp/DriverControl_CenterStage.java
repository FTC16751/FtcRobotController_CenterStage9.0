/* P3 2021-22 season Driver Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.team24030.TeleOp;

import com.qualcomm.robotcore.eventloop.EventLoop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.team24030.robot.utilities.ClawUtil;
import org.firstinspires.ftc.team24030.robot.utilities.DriveUtil2023;
import org.firstinspires.ftc.team24030.robot.utilities.LauncherUtil;


@TeleOp(name="Driver Control Center Stage", group="Teleop")
public class DriverControl_CenterStage extends LinearOpMode {
    DriveUtil2023 drive = new DriveUtil2023(this);
    ClawUtil claw = new ClawUtil(this);
    private DcMotorEx ShoulderArm;
    private DcMotorEx ElbowArm;
    int temp = 1;
    double DRIVE_SPEED = 1;
    double handOffset   = 0;

    int elbowStowPosition;
    int elbowPosition1;
    int elbowPosition2;
    int elbowPosition3;
    int elbowPosition;

    int shoulderStowPosition;
    int shoulderPosition1;
    int shoulderPosition2;
    int shoulderPosition3;
    int shoulderPosition;

    enum ArmState {
        START,
        ACQUIRE,
        PICK,
        TRANSPORT,
        SCORE,
        STOP,
        MANUAL,
        HANG_READY,
        HANG
    }
    ArmState state = ArmState.START;
    private Servo wristServo;
    private Servo leftClaw;
    private Servo rightClaw;
    // Define servo positions for wrist and claws
    private double wristPosition = 0.477; // Initial position
    private double leftClawPosition = 0.25; // Initial position
    private double rightClawPosition = 0.75; // Initial position

    // Define variables to keep track of previous bumper states
    private boolean leftBumperPrevState = false;
    private boolean rightBumperPrevState = false;
    LauncherUtil launcherUtil;
    ElapsedTime timer;
    boolean yButtonState = false;
    boolean aButtonState = false;
    @Override


    //Initialize and run program
    public void runOpMode() throws InterruptedException {
        //update status on driver station
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        //init external hardware classes
        drive.init(hardwareMap);
        claw.init();

        ShoulderArm = hardwareMap.get(DcMotorEx.class, "Shoulder");
        ElbowArm = hardwareMap.get(DcMotorEx.class, "Elbow");

        ShoulderArm.setDirection(DcMotorEx.Direction.FORWARD);
        ElbowArm.setDirection(DcMotorEx.Direction.FORWARD);

        ShoulderArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ElbowArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ShoulderArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElbowArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ShoulderArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElbowArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //ShoulderArm.setTargetPositionTolerance(2);
        ElbowArm.setTargetPosition(15);

        int ShoulderminPosition = 0;
        int ShouldermaxPosition = 500;
        shoulderStowPosition = 0;
        shoulderPosition1 = 0;
        shoulderPosition2 = 388;
        shoulderPosition3 = 1000;

        int minPosition = 0;
        int maxPosition = 3600;
        elbowStowPosition = 100;
        elbowPosition1 = 3500;
        elbowPosition2 = 0;
        elbowPosition3 = 3500;

        //default drive move to 1 (arcade)
        int driveMode = 1;
        claw.setClawOpen();

        wristServo = hardwareMap.get(Servo.class, "wristservo");
        leftClaw = hardwareMap.get(Servo.class, "leftclaw");
        rightClaw = hardwareMap.get(Servo.class, "rightclaw");

        leftClaw.setDirection(Servo.Direction.REVERSE);
        rightClaw.setDirection(Servo.Direction.REVERSE);
        // Set servo initial positions
        wristServo.setPosition(wristPosition);
        leftClaw.setPosition(leftClawPosition);
        rightClaw.setPosition(rightClawPosition);

        launcherUtil = new LauncherUtil(hardwareMap);
        timer = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
            if (gamepad2.left_bumper){
                claw.setClawOpen();
            } else {// continue looping//
            }

            if (gamepad2.right_bumper){
                claw.setClawClosed();
            }else {// continue looping
            }


            switch (state) {
                case START:
                    telemetry.addData("State", "Start");
                    stopElbow();
                    stopShoulder();

                    if (gamepad2.dpad_left) {
                        state = ArmState.ACQUIRE;
                    }
                    else if (gamepad2.dpad_up) {
                        state = ArmState.TRANSPORT;
                    }
                    else if (gamepad2.dpad_right) {
                        state = ArmState.SCORE;
                    }
                    else if (gamepad2.dpad_down) {
                        state = ArmState.STOP;
                    }
                    else if (gamepad2.back) {
                        state = ArmState.MANUAL;
                    }
                    else if (gamepad2.y) {
                        state = ArmState.HANG_READY;
                    }
                    else if (gamepad2.a) {
                        state = ArmState.HANG_READY;
                    }
                    else {}
                    break;

                case ACQUIRE:
                    telemetry.addData("State", "Acquire");
                    shoulderPosition = shoulderPosition1;
                    ShoulderArm.setTargetPosition(shoulderPosition);
                    ShoulderArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    ShoulderArm.setPower(1);

                    elbowPosition = elbowPosition1;
                    ElbowArm.setTargetPosition(elbowPosition);
                    ElbowArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    ElbowArm.setPower(1);

                    //while (ElbowArm.isBusy() ) {
                        // Waiting for both slides to reach the target position
                      //  telemetry.addData("elbow working yall ", "Yep");
                    //}
                    //ElbowArm.setPower(0);
                    if (ElbowArm.getCurrentPosition() == shoulderPosition){
                        telemetry.addData("Elbow Extended ", ElbowArm.getCurrentPosition());
                        ElbowArm.setPower(0);
                    }

                    if (gamepad2.dpad_left) {
                        state = ArmState.ACQUIRE;
                    }
                    else if (gamepad2.dpad_up) {
                        state = ArmState.TRANSPORT;
                    }
                    else if (gamepad2.dpad_right) {
                        state = ArmState.SCORE;
                    }
                    else if (gamepad2.dpad_down) {
                        state = ArmState.STOP;
                    }
                    else if (gamepad2.back) {
                        state = ArmState.MANUAL;
                    }
                    else if (gamepad2.y) {
                        state = ArmState.HANG_READY;
                    }
                    else if (gamepad2.a) {
                        state = ArmState.HANG_READY;
                    }

                    else {}
                    break;

                case PICK:
                    if (gamepad2.dpad_left) {
                        state = ArmState.ACQUIRE;
                    }
                    else if (gamepad2.dpad_up) {
                        state = ArmState.TRANSPORT;
                    }
                    else if (gamepad2.dpad_right) {
                        state = ArmState.SCORE;
                    }
                    else if (gamepad2.dpad_down) {
                        state = ArmState.STOP;
                    }
                    else if (gamepad2.y) {
                        state = ArmState.HANG_READY;
                    }
                    else if (gamepad2.a) {
                        state = ArmState.HANG_READY;
                    }
                    else {}
                    break;
                case TRANSPORT:
                    telemetry.addData("State", "Transport");
                    shoulderPosition = shoulderPosition2;
                    ShoulderArm.setTargetPosition(shoulderPosition);
                    ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ShoulderArm.setPower(1);

                    elbowPosition = elbowPosition2;
                    ElbowArm.setTargetPosition(elbowPosition);
                    ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ElbowArm.setPower(1);

                    if (gamepad2.dpad_left) {
                        state = ArmState.ACQUIRE;
                    }
                    else if (gamepad2.dpad_up) {
                        state = ArmState.TRANSPORT;
                    }
                    else if (gamepad2.dpad_right) {
                        state = ArmState.SCORE;
                    }
                    else if (gamepad2.dpad_down) {
                        state = ArmState.STOP;
                    }
                    else if (gamepad2.back) {
                        state = ArmState.MANUAL;
                    }
                    else if (gamepad2.y) {
                        state = ArmState.HANG_READY;
                    }
                    else if (gamepad2.a) {
                        state = ArmState.HANG_READY;
                    }
                    else {}

                    break;

                case SCORE:
                    telemetry.addData("State", "Score");
                    shoulderPosition = shoulderPosition3;
                    ShoulderArm.setTargetPosition(shoulderPosition);
                    ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ShoulderArm.setPower(1.0);

                    elbowPosition = elbowPosition3;
                    ElbowArm.setTargetPosition(elbowPosition);
                    ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ElbowArm.setPower(0.75);


                    if (gamepad2.dpad_left) {
                        state = ArmState.ACQUIRE;
                    }
                    else if (gamepad2.dpad_up) {
                        state = ArmState.TRANSPORT;
                    }
                    else if (gamepad2.dpad_right) {
                        state = ArmState.SCORE;
                    }
                    else if (gamepad2.dpad_down) {
                        state = ArmState.STOP;
                    }
                    else if (gamepad2.back) {
                        state = ArmState.MANUAL;
                    }
                    else if (gamepad2.y) {
                        state = ArmState.HANG_READY;
                    }
                    else if (gamepad2.a) {
                        state = ArmState.HANG_READY;
                    }
                    else {}

                    break;
                case STOP:
                    telemetry.addData("State", "Stop");
                    elbowPosition = elbowStowPosition;
                    ElbowArm.setTargetPosition(elbowPosition);
                    ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ElbowArm.setPower(.75);

                    shoulderPosition = shoulderStowPosition;
                    ShoulderArm.setTargetPosition(shoulderPosition);
                    ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ShoulderArm.setPower(.25);

                    if (ElbowArm.getCurrentPosition() <= 10){
                        telemetry.addData("Elbow Retracted ", ElbowArm.getCurrentPosition());
                        ElbowArm.setPower(0);
                        ElbowArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    }

                    if (ShoulderArm.getCurrentPosition() <= 0){
                        telemetry.addData("Shoulder Retracted ", ShoulderArm.getCurrentPosition());
                        ShoulderArm.setPower(0);
                        ShoulderArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    }
                    //while (ShoulderArm.isBusy()) {
                        // Waiting for both slides to reach the target position
                       // telemetry.addData("shoulder busy ", "Yep");
                    //}
                    telemetry.addData("shoulder busy ", "Nope");

                    //ShoulderArm.setPower(0);
                    //resetEncoder();
                    if (gamepad2.start) {
                        state = ArmState.START;
                    }
                    else if (gamepad2.dpad_left) {
                        state = ArmState.ACQUIRE;
                    }
                    else if (gamepad2.dpad_up) {
                        state = ArmState.TRANSPORT;
                    }
                    else if (gamepad2.dpad_right) {
                        state = ArmState.SCORE;
                    }
                    else if (gamepad2.dpad_down) {
                        state = ArmState.STOP;
                    }
                    else if (gamepad2.back) {
                        state = ArmState.MANUAL;
                    }
                    else if (gamepad2.y) {
                        state = ArmState.HANG_READY;
                    }
                    else if (gamepad2.a) {
                        state = ArmState.HANG_READY;
                    }
                    else {}

                    break;
                case MANUAL:
                    telemetry.addData("State", "Manual");
                    //ElbowArm.setPower(-gamepad2.left_stick_y);
                    if (gamepad2.x) {
                        int currentPosition;
                        int newPosition;
                        currentPosition = ElbowArm.getCurrentPosition();
                        newPosition = currentPosition - 100;
                        ElbowArm.setTargetPosition(newPosition);
                        ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ElbowArm.setPower(0.7);
                    } else if (gamepad2.b) {
                        int currentPosition;
                        int newPosition;
                        currentPosition = ElbowArm.getCurrentPosition();
                        newPosition = currentPosition + 100;
                        ElbowArm.setTargetPosition(newPosition);
                        ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ElbowArm.setPower(0.7);
                    } else {
                        ElbowArm.setPower(0);
                    }
                    if (gamepad2.start) {
                        state = ArmState.START;
                    } else if (gamepad2.dpad_left) {
                    state = ArmState.ACQUIRE;
                    }
                    else if (gamepad2.dpad_up) {
                    state = ArmState.TRANSPORT;
                    }
                    else if (gamepad2.dpad_right) {
                    state = ArmState.SCORE;
                    }
                    else if (gamepad2.dpad_down) {
                    state = ArmState.STOP;
                    }
                    else if (gamepad2.back) {
                    state = ArmState.MANUAL;
                    }
                    else if (gamepad2.y) {
                        state = ArmState.HANG_READY;
                    }
                    else if (gamepad2.a) {
                        state = ArmState.HANG;
                    }
                    break;

                case HANG_READY:
                    telemetry.addData("State", "Hang Ready");
                    shoulderPosition = 1990;
                    ShoulderArm.setTargetPosition(shoulderPosition);
                    ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ShoulderArm.setPower(1.0);

                    elbowPosition = 3500;
                    ElbowArm.setTargetPosition(elbowPosition);
                    ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ElbowArm.setPower(0.75);


                    if (gamepad2.dpad_left) {
                        state = ArmState.ACQUIRE;
                    }
                    else if (gamepad2.dpad_up) {
                        state = ArmState.TRANSPORT;
                    }
                    else if (gamepad2.dpad_right) {
                        state = ArmState.SCORE;
                    }
                    else if (gamepad2.dpad_down) {
                        state = ArmState.STOP;
                    }
                    else if (gamepad2.back) {
                        state = ArmState.MANUAL;
                    }
                    else if (gamepad2.y) {
                        state = ArmState.HANG_READY;
                    }
                    else if (gamepad2.a) {
                        state = ArmState.HANG;
                    }
                    else {}

                    break;
                case HANG:
                    telemetry.addData("State", "Hang");

                    elbowPosition = 100;
                    ElbowArm.setTargetPosition(elbowPosition);
                    ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ElbowArm.setPower(0.75);
                    while (ElbowArm.isBusy()) {

                    }
                    shoulderPosition = 1000;
                    ShoulderArm.setTargetPosition(shoulderPosition);
                    ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ShoulderArm.setPower(1.0);

                    if (gamepad2.dpad_left) {
                        state = ArmState.ACQUIRE;
                    }
                    else if (gamepad2.dpad_up) {
                        state = ArmState.TRANSPORT;
                    }
                    else if (gamepad2.dpad_right) {
                        state = ArmState.SCORE;
                    }
                    else if (gamepad2.dpad_down) {
                        state = ArmState.STOP;
                    }
                    else if (gamepad2.back) {
                        state = ArmState.MANUAL;
                    }
                    else if (gamepad2.y) {
                        state = ArmState.HANG_READY;
                    }
                    else if (gamepad2.a) {
                        state = ArmState.HANG;
                    }
                    else {}

                    break;
                default:
                    telemetry.addData("State", "Default");
                    break;

            }



            // Control wrist using triggers
            double wristPower = 0.0;
            if (gamepad2.left_trigger > 0) {
                wristPosition -= 0.001; // Change increment for desired movement speed
                wristPosition = Math.max(0.0, wristPosition);
            } else if (gamepad2.right_trigger > 0) {
                wristPosition += 0.001; // Change increment for desired movement speed
                wristPosition = Math.min(1.0, wristPosition);
            }
            wristServo.setPosition(wristPosition);

            // Control claws using bumpers (toggle open/close with Falling Edge Detector)
            if (gamepad2.left_bumper && !leftBumperPrevState) {
                //1 is open .x is close
                leftClawPosition = (leftClawPosition == 0.5) ? 0.25 : 0.5;
                leftClaw.setPosition(leftClawPosition);
            }
            if (gamepad2.right_bumper && !rightBumperPrevState) {
                rightClawPosition = (rightClawPosition == 0.55) ? 0.75 : 0.55;
                rightClaw.setPosition(rightClawPosition);
            }

            // Update previous bumper states
            leftBumperPrevState = gamepad2.left_bumper;
            rightBumperPrevState = gamepad2.right_bumper;


            // Y button rising edge detection
            if (gamepad1.y && !launcherUtil.getToggleState()) {
                launcherUtil.toggleServo();
            }

            // A button rising edge detection
            if (gamepad1.a && !aButtonState) {
                aButtonState = true;
                if (!launcherUtil.isLauncherMoving()) {
                    if (launcherUtil.getLauncherAngleServoPosition() == launcherUtil.LAUNCHER_ANGLE_DOWN_POSITION) {
                        launcherUtil.raiseLauncher();
                    } else {
                        launcherUtil.lowerLauncherDown();
                    }
                }
            } else if (!gamepad1.a) {
                aButtonState = false;
            }
            //telemetry
            telemetry.addData("State", state);
            telemetry.addData("elbow current position", ElbowArm.getCurrentPosition());
            telemetry.addData("Shoulder current position", ShoulderArm.getCurrentPosition());
            telemetry.addData("ShoulderArm.isBusy() ", ShoulderArm.isBusy());
            telemetry.addData("ElbowArm.isBusy() ", ElbowArm.isBusy());
            telemetry.addData("wrist position", wristServo.getPosition());
            telemetry.addData("left claw position", leftClaw.getPosition());
            telemetry.addData("right claw position", rightClaw.getPosition());
            telemetry.update();

        } //end OpModeIsActive
    }  //end runOpMode

    private void stopShoulder() {
        ShoulderArm.setPower(0);
    }

    private void stopElbow() {
        ElbowArm.setPower(0);
    }
    public void resetEncoder(){
        ShoulderArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElbowArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void tankDrive() {
        drive.tankDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, DRIVE_SPEED);
    }

    public void arcadeDrive() {
        drive.arcadeDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, DRIVE_SPEED);
    }

} //end program
