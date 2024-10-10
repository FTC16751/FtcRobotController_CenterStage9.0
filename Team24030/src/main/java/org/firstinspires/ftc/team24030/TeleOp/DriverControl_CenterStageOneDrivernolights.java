/* P3 2021-22 season Driver Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.team24030.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.team24030.robot.utilities.DriveUtil2023;

import java.util.concurrent.TimeUnit;


@TeleOp(name="Driver Control One Driver no lights", group="Teleop")
public class DriverControl_CenterStageOneDrivernolights extends LinearOpMode {
    Deadline gamepadRateLimit = new Deadline(250, TimeUnit.MILLISECONDS);
    DriveUtil2023 drive = new DriveUtil2023(this);
   // ClawUtil claw = new ClawUtil(this);
    private DcMotorEx ShoulderArm;
    private DcMotorEx ElbowArm;
    int temp = 1;
    double DRIVE_SPEED = 1.0;
    double handOffset   = 0;

    int elbowStowPosition;
    int elbowPosition1_acquire;
    int elbowPosition2_transport;
    int elbowPosition3_score;
    int elbowPosition;
    int elbowPositionPull;

    int shoulderStowPosition;
    int shoulderPosition1_acquire;
    int shoulderPosition2_transport;
    int shoulderPosition3_score;
    int shoulderPosition;
    int shoulderPositionPull;

    enum ArmState {
        START,
        ACQUIRE,
        PICK,
        TRANSPORT,
        SCORE,
        SCORE_BACK_LOW,
        SCORE_BACK_MID,
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
    private double wristPosition = 1.0;//0.40; // Initial position

    private static final double LEFT_CLAW_OPEN_POSITION = 0.25;
    private static final double LEFT_CLAW_CLOSED_POSITION = 0.5;
    private static final double RIGHT_CLAW_OPEN_POSITION = 0.75;
    private static final double RIGHT_CLAW_CLOSED_POSITION = .55;
    private double leftClawPosition = LEFT_CLAW_CLOSED_POSITION; // Initial position
    private double rightClawPosition = RIGHT_CLAW_CLOSED_POSITION; // Initial position

    // Define variables to keep track of previous bumper states
    private boolean leftBumperPrevState = false;
    private boolean rightBumperPrevState = false;
    ElapsedTime timer;
    boolean yButtonState = false;
    boolean aButtonState = false;
    int driveMode;
    NormalizedColorSensor lcolorSensor, rcolorSensor;
    @Override


    //Initialize and run program
    public void runOpMode() throws InterruptedException {
        //update status on driver station
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        initializeHardware();
        setArmPositionValues();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /***************************************************************
         * Game Execution Area
         * Runs continous until "STOP" pressed on Driver Station
         ***************************************************************/
        while (opModeIsActive()) {
            doDriver1Controls();
            doClawControls();
            doArmControlStateMachine();
            doWristControls();
           // doLauncherControl();
            doTelemetry();

        } //end OpModeIsActive
    }  //end runOpMode


    private void doTelemetry() {            //telemetry
        telemetry.addData("State", state);
        telemetry.addData("elbow current position", ElbowArm.getCurrentPosition());
        telemetry.addData("Shoulder current position", ShoulderArm.getCurrentPosition());
        telemetry.addData("ShoulderArm.isBusy() ", ShoulderArm.isBusy());
        telemetry.addData("ElbowArm.isBusy() ", ElbowArm.isBusy());
        telemetry.addData("wrist position", wristServo.getPosition());
        telemetry.addData("left claw position", leftClaw.getPosition());
        telemetry.addData("right claw position", rightClaw.getPosition());
        telemetry.update();
    }


    private void doWristControls() {
        // Control wrist using triggers
        double wristPower = 0.0;
        if (gamepad1.left_trigger > 0 && gamepad1.left_trigger <1.0) {
            wristPosition += 0.005; // Change increment for desired movement speed
            wristPosition = Math.max(0.0, wristPosition);
        } else if (gamepad1.left_trigger == 1.0) {
            wristPosition += 0.005; // Change increment for desired movement speed
            wristPosition = Math.max(0.0, wristPosition);
        } else if (gamepad1.right_trigger > 0 && gamepad1.right_trigger <1.0) {
            wristPosition -= 0.005; // Change increment for desired movement speed
            wristPosition = Math.min(1.0, wristPosition);
        }else if (gamepad1.right_trigger ==1.0) {
            wristPosition -= 0.005; // Change increment for desired movement speed
            wristPosition = Math.min(1.0, wristPosition);
        }
        wristServo.setPosition(wristPosition);
    }

    private void initializeHardware() {
        //init external hardware classes
        drive.init(hardwareMap);
        //claw.init();

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
        ElbowArm.setTargetPosition(0);

       // claw.setClawClosed();

        wristServo = hardwareMap.get(Servo.class, "wristservo");
        leftClaw = hardwareMap.get(Servo.class, "leftclaw");
        rightClaw = hardwareMap.get(Servo.class, "rightclaw");

        leftClaw.setDirection(Servo.Direction.REVERSE);
        rightClaw.setDirection(Servo.Direction.REVERSE);
        // Set servo initial positions
        wristServo.setPosition(wristPosition);
        leftClaw.setPosition(leftClawPosition);
        rightClaw.setPosition(rightClawPosition);


        timer = new ElapsedTime();
        //default drive move to 1 (arcade)
        driveMode = 1;

      //  lcolorSensor = hardwareMap.get(NormalizedColorSensor.class, "lcolorsensor");
       // rcolorSensor = hardwareMap.get(NormalizedColorSensor.class, "rcolorsensor");
    }

    private void doDriver1Controls() {
        //Set driver speed as a percentage of full (normally set to full)
     /*   if (gamepad1.right_bumper & gamepad1.left_bumper) {
            DRIVE_SPEED = .25;
        } else if (gamepad1.left_bumper) {
            DRIVE_SPEED = .50;
        } else if (gamepad1.right_bumper) {
            DRIVE_SPEED = 1.00;
        } else {
            DRIVE_SPEED = .80;
        }


      */
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
    }

    private void setArmPositionValues() {
        int ShoulderminPosition = 0;
        int ShouldermaxPosition = 500;
        shoulderStowPosition = 0;
        shoulderPosition1_acquire = 306;
        shoulderPosition2_transport = 0;
        shoulderPosition3_score = 1700;
        shoulderPositionPull = 1900;

        int minPosition = 0;
        int maxPosition = 3600;
        elbowStowPosition = 0;
        elbowPosition1_acquire = 3600;
        elbowPosition2_transport = 0;
        elbowPosition3_score = 3500;
        elbowPositionPull = 0;
    }


    private void doClawControls() {
        // Control claws using bumpers (toggle open/close with Falling Edge Detector)
        if (gamepad1.left_bumper && !leftBumperPrevState) {
            //1 is open .x is close
            leftClawPosition = (leftClawPosition == LEFT_CLAW_CLOSED_POSITION) ?LEFT_CLAW_OPEN_POSITION : LEFT_CLAW_CLOSED_POSITION;
            leftClaw.setPosition(leftClawPosition);
        }
        if (gamepad1.right_bumper && !rightBumperPrevState) {
            rightClawPosition = (rightClawPosition == RIGHT_CLAW_CLOSED_POSITION) ? RIGHT_CLAW_OPEN_POSITION : RIGHT_CLAW_CLOSED_POSITION;
            rightClaw.setPosition(rightClawPosition);
        }

        // Update previous bumper states
        leftBumperPrevState = gamepad1.left_bumper;
        rightBumperPrevState = gamepad1.right_bumper;

    }

    public void closeLeftClaw() {
        leftClaw.setPosition(LEFT_CLAW_CLOSED_POSITION);
        leftBumperPrevState = !leftBumperPrevState;
    }

    public void closeRightClaw() {
        rightClaw.setPosition(RIGHT_CLAW_CLOSED_POSITION);
        rightBumperPrevState = !rightBumperPrevState;
    }

    public void openLeftClaw() {
        leftClaw.setPosition(LEFT_CLAW_OPEN_POSITION);
    }

    public void openRightClaw() {
        rightClaw.setPosition(RIGHT_CLAW_OPEN_POSITION);
    }

    private void stopShoulder() {
        ShoulderArm.setPower(0);
    }

    private void stopElbow() {
        ElbowArm.setPower(0);
    }
    public void resetEncoder(){
        ShoulderArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElbowArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShoulderArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElbowArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void tankDrive() {
        drive.tankDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, DRIVE_SPEED);
    }

    public void arcadeDrive() {
        drive.arcadeDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.right_stick_y, DRIVE_SPEED);
    }

    private void doArmControlStateMachine() {
        switch (state) {
            case START:
                telemetry.addData("State", "Start");
                stopElbow();
                stopShoulder();
                checkGamepadforNextState();
                break;

            case ACQUIRE:
                telemetry.addData("State", "Acquire");
                shoulderPosition = shoulderPosition1_acquire;
                elbowPosition = elbowPosition1_acquire;
                executeArmRunToPosition(ShoulderArm,shoulderPosition,1.0);
                executeArmRunToPosition(ElbowArm, elbowPosition, 1.0);
                if (ElbowArm.getCurrentPosition() == shoulderPosition){
                    telemetry.addData("Elbow Extended ", ElbowArm.getCurrentPosition());
                    ElbowArm.setPower(0);
                }

                checkGamepadforNextState();
                break;

            case PICK:
                checkGamepadforNextState();
                break;

            case TRANSPORT:
                telemetry.addData("State", "Transport");
                shoulderPosition = shoulderPosition2_transport;
                elbowPosition = elbowPosition2_transport;
                executeArmRunToPosition(ElbowArm, elbowPosition, 1.0);


                if (ElbowArm.getCurrentPosition() <= 10) {
                    executeArmRunToPosition(ShoulderArm, shoulderPosition, 1.0);
                }

                checkGamepadforNextState();
                break;

            case SCORE:
                telemetry.addData("State", "Score");
                shoulderPosition = shoulderPosition3_score;
                elbowPosition = elbowPosition3_score;
                executeArmRunToPosition(ShoulderArm,shoulderPosition, 1.0);
                executeArmRunToPosition(ElbowArm, elbowPosition,.75);
                checkGamepadforNextState();
                break;
            case SCORE_BACK_LOW:
                telemetry.addData("State", "Score Back Low");
                shoulderPosition = 2731;
                elbowPosition = 0;
                executeArmRunToPosition(ShoulderArm,shoulderPosition, 1.0);
                executeArmRunToPosition(ElbowArm, elbowPosition,.75);
                checkGamepadforNextState();
                break;
            case SCORE_BACK_MID:
                telemetry.addData("State", "Score Back Low");
                shoulderPosition = 2731;
                elbowPosition = 3024;
                executeArmRunToPosition(ShoulderArm,shoulderPosition, 1.0);
                executeArmRunToPosition(ElbowArm, elbowPosition,.75);
                checkGamepadforNextState();
                break;
            case STOP:
                /*for (int i = 0; i <=1 ; i++) {
                    telemetry.addData("i", i);
                    closeLeftClaw();
                    closeRightClaw();
                }*/
                telemetry.addData("State", "Stop");
                elbowPosition = elbowPositionPull;
                shoulderPosition = shoulderPositionPull;
                executeArmRunToPosition(ShoulderArm,shoulderPosition, .25);
                executeArmRunToPosition(ElbowArm, elbowPosition,.75);

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

                checkGamepadforNextState();
                break;
            case MANUAL:
                telemetry.addData("State", "Manual");
                //ElbowArm.setPower(-gamepad2.left_stick_y);
                if (gamepad1.dpad_down) {
                    int currentPosition;
                    int newShoulderPosition;
                    currentPosition = ShoulderArm.getCurrentPosition();
                    newShoulderPosition = currentPosition - 50;
                    executeArmRunToPosition(ShoulderArm, newShoulderPosition,.7);
                } else if (gamepad1.dpad_up) {
                    int currentPosition;
                    int newShoulderPosition;
                    currentPosition = ShoulderArm.getCurrentPosition();
                    newShoulderPosition = currentPosition + 50;
                    executeArmRunToPosition(ShoulderArm, newShoulderPosition,.7);

                } else {}
                if (gamepad1.dpad_right) {
                    int currentPosition;
                    int newElbowPosition;
                    currentPosition = ElbowArm.getCurrentPosition();
                    newElbowPosition = currentPosition - 100;
                    executeArmRunToPosition(ElbowArm, newElbowPosition, 1.0);
                } else if (gamepad1.dpad_left) {
                    int currentPosition;
                    int newElbowPosition;
                    currentPosition = ElbowArm.getCurrentPosition();
                    newElbowPosition = currentPosition + 100;
                    executeArmRunToPosition(ElbowArm, newElbowPosition, 1.0);

                } else {}
                checkGamepadforNextState();
                /*
                if (gamepad2.start) {
                    resetEncoder();
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
                }  */
                break;

            case HANG_READY:
                telemetry.addData("State", "Hang Ready");
                shoulderPosition = 1990;
                elbowPosition = 3500;
                executeArmRunToPosition(ShoulderArm,shoulderPosition, 1.0);
                executeArmRunToPosition(ElbowArm, elbowPosition,.75);
                checkGamepadforNextState();
                break;
            case HANG:
                telemetry.addData("State", "Hang");

                elbowPosition = 100;
                shoulderPosition = 1000;
                executeArmRunToPosition(ShoulderArm,shoulderPosition, 1.0);
                while (ElbowArm.isBusy()) {

                }
                executeArmRunToPosition(ElbowArm, elbowPosition,.75);
                checkGamepadforNextState();
                break;

            default:
                telemetry.addData("State", "Default");
                break;

        }
    }

    private void executeArmRunToPosition(DcMotorEx motor, int position,double power) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    private void checkGamepadforNextState() {
        if (gamepad1.x){//dpad_left) {
            state = ArmState.TRANSPORT;
        }
        else if (gamepad1.y){//dpad_up) {
            state = ArmState.SCORE;
        }
        else if (gamepad1.b){//dpad_right) {
            state = ArmState.ACQUIRE;
        }
        else if (gamepad1.a){//dpad_down) {
            state = ArmState.STOP;
        }
        else if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            state = ArmState.MANUAL;
        }
/*
        else if (gamepad1.y) {
            state = ArmState.HANG_READY;
        }
        else if (gamepad1.a) {
            state = ArmState.HANG;
        }


        else {} */
    }

} //end program
