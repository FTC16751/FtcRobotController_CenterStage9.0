package org.firstinspires.ftc.team24030.Autonomous.UnderDevelopment;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team24030.robot.utilities.ArmUtil;
import org.firstinspires.ftc.team24030.robot.utilities.DriveUtil2023;
import org.firstinspires.ftc.team24030.robot.utilities.TeamElementSubsystem;

@Autonomous(name="Red BackDrop Vision", group="Auto")

public class RedBackStageVision extends LinearOpMode {

    public int element_zone = 1;

    private TeamElementSubsystem teamElementDetection = null;

    boolean togglePreview = true;
    private DriveUtil2023 driveUtil2023 = null;
    ArmUtil armUtil = new ArmUtil(this);
    private Servo wristServo;
    private Servo leftClaw;
    private Servo rightClaw;
    // Define servo positions for wrist and claws
    private double wristPosition = 0.6; // Initial position
    private double leftClawPosition = 1.0; // Initial position - close
    private double rightClawPosition = 0.0; // Initial position - close

    private DcMotorEx ShoulderArm;
    private DcMotorEx ElbowArm;
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

    //Initializing Hardware
    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        teamElementDetection = new TeamElementSubsystem(hardwareMap);
        driveUtil2023 = new DriveUtil2023(this);
        driveUtil2023.init(hardwareMap);
        armUtil.init(hardwareMap);
        telemetry.addData("Object Creation", "Done");
        telemetry.update();

        wristServo = hardwareMap.get(Servo.class, "wristservo");
        leftClaw = hardwareMap.get(Servo.class, "leftclaw");
        rightClaw = hardwareMap.get(Servo.class, "rightclaw");

        leftClaw.setDirection(Servo.Direction.REVERSE);
        rightClaw.setDirection(Servo.Direction.REVERSE);
        // Set servo initial positions
        wristServo.setPosition(wristPosition);
        leftClaw.setPosition(leftClawPosition);
        rightClaw.setPosition(rightClawPosition);
    }

//Output on driver control hub with element zone, camera stream, and alliance color
    public void runOpMode() {

        HardwareStart();

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
        shoulderPosition3 = 800;

        int minPosition = 0;
        int maxPosition = 3600;
        elbowStowPosition = 100;
        elbowPosition1 = 3500;
        elbowPosition2 = 0;
        elbowPosition3 = 1000;

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
        String curAlliance = "red";

        while (!opModeIsActive() && !isStopRequested()) {
            element_zone = teamElementDetection.elementDetection(telemetry);
            if (togglePreview && gamepad2.a) {
                togglePreview = false;
                teamElementDetection.toggleAverageZone(gamepad2);
            } else if (!gamepad2.a) {
                togglePreview = true;
            }



            teamElementDetection.setAlliance(curAlliance);
            telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
            telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());
            telemetry.addData("Object", "Passed waitForStart");
            telemetry.update();

        }
      //  while(opModeIsActive()&& !isStopRequested()){
        waitForStart();
        //wristServo.setPosition(wristPosition);
            if(element_zone==1){
                driveUtil2023.driveRobotDistanceForward(75,0.5);
                driveUtil2023.rotateLeft90Degrees();
                leftClaw.setPosition(0.2);
                sleep(2000);
                driveUtil2023.driveRobotDistanceBackward(10,.5);
                driveUtil2023.rotateLeft180Degrees();
                //driveUtil2023.driveRobotDistanceStrafeLeft(65,.5);
                driveUtil2023.driveRobotDistanceForward(70,.5);


                telemetry.addData("State", "Score");
                shoulderPosition = 800;
                ShoulderArm.setTargetPosition(shoulderPosition);
                ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ShoulderArm.setPower(1.0);

                elbowPosition = 2000;
                ElbowArm.setTargetPosition(elbowPosition);
                ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElbowArm.setPower(1.0);

                while (ShoulderArm.isBusy() && ElbowArm.isBusy()){
                    //hello
                }
                //driveUtil2023.driveRobotDistanceStrafeLeft(15,.25);
                sleep(1500);
                driveUtil2023.driveRobotDistanceForward(7,.15);
                sleep(500);
                rightClaw.setPosition(1.0);
                sleep(250);
                driveUtil2023.driveRobotDistanceBackward(20,.25);
                sleep(500);
                shoulderPosition = shoulderPosition3;
                ShoulderArm.setTargetPosition(shoulderStowPosition);
                ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ShoulderArm.setPower(1.0);

                elbowPosition = elbowPosition3;
                ElbowArm.setTargetPosition(elbowStowPosition);
                ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElbowArm.setPower(1.0);

                while (ShoulderArm.isBusy() && ElbowArm.isBusy()){
                    //hello
                }
                driveUtil2023.driveRobotDistanceStrafeRight(80,.5);
                driveUtil2023.driveRobotDistanceForward(30,.5);
                sleep(2500);
                //armUtil.raiseToPosition(3,0.5);
                //armUtil.raiseToPosition(0,0.5);


            }
            else if(element_zone==2){
                driveUtil2023.driveRobotDistanceForward(65,0.5);
                leftClaw.setPosition(0.2);
                sleep(2000);
                driveUtil2023.driveRobotDistanceBackward(10,.5);
                driveUtil2023.rotateRight90Degrees();
                //
                driveUtil2023.driveRobotDistanceForward(80,.5);


                telemetry.addData("State", "Score");
                shoulderPosition = 500;
                ShoulderArm.setTargetPosition(shoulderPosition);
                ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ShoulderArm.setPower(1.0);

                elbowPosition = 2000;
                ElbowArm.setTargetPosition(elbowPosition);
                ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElbowArm.setPower(1.0);

                while (ShoulderArm.isBusy() && ElbowArm.isBusy()){
                    //hello
                }
                //driveUtil2023.driveRobotDistanceStrafeLeft(15,.25);
                sleep(1500);
                driveUtil2023.driveRobotDistanceStrafeLeft(30,.5);
                driveUtil2023.driveRobotDistanceForward(7,.15);
                sleep(500);
                rightClaw.setPosition(0.8);
                sleep(250);
                driveUtil2023.driveRobotDistanceBackward(20,.25);
                sleep(500);
                shoulderPosition = shoulderStowPosition;
                ShoulderArm.setTargetPosition(0);
                ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ShoulderArm.setPower(1.0);

                elbowPosition = elbowStowPosition;
                ElbowArm.setTargetPosition(0);
                ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElbowArm.setPower(1.0);

                while (ShoulderArm.isBusy() && ElbowArm.isBusy()){
                    //hello
                }
                driveUtil2023.driveRobotDistanceStrafeRight(70,.5);
                driveUtil2023.driveRobotDistanceForward(30,.5);
                sleep(2500);

            }
            else if (element_zone==3){
                //drive robot to spike marks
                driveUtil2023.driveRobotDistanceForward(90,0.5);
                //move the custom game element out of the way
                driveUtil2023.driveRobotDistanceStrafeRight(30,.5);
                //  driveUtil2023.driveRobotDistanceBackward(20,.5);
                //come back and drop the purple game piece
                driveUtil2023.driveRobotDistanceStrafeLeft(5,.5);
                driveUtil2023.driveRobotDistanceBackward(30,0.5);
                leftClaw.setPosition(0.20);
                sleep(500);
                //drive path towards the backdrop
                driveUtil2023.driveRobotDistanceBackward(20,.5);
                driveUtil2023.rotateRight90Degrees();
                //drive to the backdrop
                driveUtil2023.driveRobotDistanceForward(60,.5);

                //set the arm to score (raise shoulder, then extend elbow)
                telemetry.addData("State", "Score");
                shoulderPosition = 800;
                ShoulderArm.setTargetPosition(shoulderPosition);
                ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ShoulderArm.setPower(1.0);

                elbowPosition = 2000;
                ElbowArm.setTargetPosition(elbowPosition);
                ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElbowArm.setPower(0.75);

                wristServo.setPosition(0.6);

                while (ShoulderArm.isBusy() && ElbowArm.isBusy()){
                    //hello
                }

                //make some fine tuned movements to the backdrop
                driveUtil2023.driveRobotDistanceStrafeLeft(20,.25);
                driveUtil2023.driveRobotDistanceForward(7,.15);
                sleep(500);

                //open claw to drop yellow game piece
                rightClaw.setPosition(0.8);
                sleep(250);

                //move away from backdrop, lower arm and park in corner
                driveUtil2023.driveRobotDistanceBackward(20,.25);
                sleep(500);
                shoulderPosition = shoulderPosition3;
                ShoulderArm.setTargetPosition(0);
                ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ShoulderArm.setPower(.5);

                elbowPosition = elbowPosition3;
                ElbowArm.setTargetPosition(0);
                ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElbowArm.setPower(0.75);

                while (ShoulderArm.isBusy() && ElbowArm.isBusy()){
                    //hello
                }

                //go park
                driveUtil2023.driveRobotDistanceStrafeRight(35,.5);
                driveUtil2023.driveRobotDistanceForward(30,.5);
            } //strafe over more at the end and go forward more before dropping the claw
            else {
        }
        //}
        }
    }

