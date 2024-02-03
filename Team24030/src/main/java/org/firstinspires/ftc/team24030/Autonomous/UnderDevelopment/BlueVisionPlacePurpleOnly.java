package org.firstinspires.ftc.team24030.Autonomous.UnderDevelopment;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team24030.robot.utilities.ArmUtil;
import org.firstinspires.ftc.team24030.robot.utilities.DriveUtil2023;
import org.firstinspires.ftc.team24030.robot.utilities.TeamElementSubsystem;

@Autonomous(name="Blue Place Purple Pixel ONLY", group="BLUE")

public class BlueVisionPlacePurpleOnly extends LinearOpMode {

    public int element_zone = 1;

    private TeamElementSubsystem teamElementDetection = null;

    boolean togglePreview = true;
    private DriveUtil2023 driveUtil2023 = null;
    ArmUtil armUtil = new ArmUtil(this);
    private Servo wristServo;
    private Servo leftClaw;
    private Servo rightClaw;
    // Define servo positions for wrist and claws
    private double wristPosition = 0.4; // Initial position
    private double leftClawPosition = LEFT_CLAW_CLOSED_POSITION; // Initial position - close
    private double rightClawPosition = RIGHT_CLAW_CLOSED_POSITION; // Initial position - close

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
    private static final double LEFT_CLAW_OPEN_POSITION = 0.25;
    private static final double LEFT_CLAW_CLOSED_POSITION = 0.5;
    private static final double RIGHT_CLAW_OPEN_POSITION = 0.75;
    private static final double RIGHT_CLAW_CLOSED_POSITION = .55;
    private String curAlliance = "blue";

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
        elbowPosition3 = 1500;

        telemetry.addData("Object Creation", "Done");
        telemetry.update();

    }

//Output on driver control hub with element zone, camera stream, and alliance color
    public void runOpMode() {

        HardwareStart();


        while (!opModeIsActive() && !isStopRequested()) {
            element_zone = teamElementDetection.elementDetection(telemetry);
            if (togglePreview && gamepad2.a) {
                togglePreview = false;
                teamElementDetection.toggleAverageZone(gamepad2);
            } else if (!gamepad2.a) {
                togglePreview = true;
            }

            teamElementDetection.setAlliance(curAlliance);
            telemetry.addData("Object", "Passed waitForStart");
            telemetry.update();

        }
      //  while(opModeIsActive()&& !isStopRequested()){
        waitForStart();

            if(element_zone==1){
                //drive robot to spike marks
                driveUtil2023.driveRobotDistanceForward(90,0.5);
                //move the custom game element out of the way
                driveUtil2023.driveRobotDistanceStrafeLeft(30,.5);
              //  driveUtil2023.driveRobotDistanceBackward(20,.5);
                //come back and drop the purple game piece
                driveUtil2023.driveRobotDistanceStrafeRight(5,.5);
                driveUtil2023.driveRobotDistanceBackward(30,0.5);
                leftClaw.setPosition(LEFT_CLAW_OPEN_POSITION);
                sleep(500);
                //drive path towards the backdrop
                driveUtil2023.driveRobotDistanceBackward(20,.5);

                shoulderPosition = shoulderStowPosition;
                ShoulderArm.setTargetPosition(0);
                ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ShoulderArm.setPower(1.0);

                elbowPosition = elbowStowPosition;
                ElbowArm.setTargetPosition(0);
                ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElbowArm.setPower(0.75);

                while (ShoulderArm.isBusy() && ElbowArm.isBusy()){
                    //hello
                }

            }
            else if(element_zone==2){
               //copied from red zone 2
                driveUtil2023.driveRobotDistanceForward(68,0.5);
                leftClaw.setPosition(LEFT_CLAW_OPEN_POSITION);
                sleep(500);
                driveUtil2023.driveRobotDistanceBackward(10,.5);

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

            }
            else if (element_zone==3){
                //copied from red zone 1
                driveUtil2023.driveRobotDistanceForward(75,0.5);
                driveUtil2023.rotateRight90Degrees();
                leftClaw.setPosition(LEFT_CLAW_OPEN_POSITION);
                sleep(1000);
                driveUtil2023.driveRobotDistanceBackward(10,.5);

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
            }
            else {
        }
        //}
        }
    }

