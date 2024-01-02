package org.firstinspires.ftc.team23469.Autonomous.Learning;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team23469.robot.utilities.Learning.TeamElementSubsystem;
import org.firstinspires.ftc.team23469.robot.utilities.Production.ClawUtil;
import org.firstinspires.ftc.team23469.robot.utilities.Production.DriveUtil;
import org.firstinspires.ftc.team23469.robot.utilities.Production.LinearSlidesUtil;

@Autonomous(name="Auto With Vision Red Template", group="Red Auto")

public class AutoWithVisionRed_Template extends LinearOpMode {
    public int element_zone = 1;
    private TeamElementSubsystem teamElementDetection = null;
    boolean togglePreview = true;
    private DriveUtil driveUtil = null;
    private ClawUtil claw = null;
    private LinearSlidesUtil slides = null;

//Initializing Hardware
    public void HardwareStart() {
        telemetry.addData("Initilizing Hardware", "Start");
        telemetry.update();

        //initilize the camera utiity and vision pipeline
        teamElementDetection = new TeamElementSubsystem(hardwareMap);

        //initilize the drive train
        driveUtil = new DriveUtil(this);
        driveUtil.init(hardwareMap);

        //initialize the claw/wrist
        claw = new ClawUtil(hardwareMap);

        //initialize the slides
        slides = new LinearSlidesUtil(hardwareMap);

        //set things up in initiual state for autonomous (open/closed,set positions, etc)
        //example
        claw.closeClaw();
        claw.raiseWrist();
    }

//Output on driver control hub with element zone, camera stream, and alliance color
    public void runOpMode() {
        //initilize hardware
        HardwareStart();

        telemetry.addData("Hardware Initlized", "Done");
        telemetry.update();

        //set the alliance color (red or blue)
        String curAlliance = "red";


        //loop through the following while opmode is active:
        while (!opModeIsActive() && !isStopRequested()) {
            element_zone = teamElementDetection.elementDetection(telemetry);
            if (togglePreview && gamepad2.a) {
                togglePreview = false;
                teamElementDetection.toggleAverageZone(gamepad2);
            } else if (!gamepad2.a) {
                togglePreview = true;
            }
            teamElementDetection.setAlliance(curAlliance);
            telemetry.addData("Object", "WaitForStart");
            telemetry.update();


        }

        waitForStart();
            if(element_zone==1){
                telemetry.addData("Zone 1 chosen", element_zone);
                telemetry.update();
                driveUtil.driveRobotDistanceForward(50, .3); //score p pixel
                driveUtil.rotateLeft90Degrees();
                driveUtil.driveRobotDistanceForward(16,.3);
                slides.moveToPosition(200);
                sleep(250);
                driveUtil.driveRobotDistanceBackward(20,.3);
                driveUtil.rotateRight90Degrees();
                driveUtil.rotateRight90Degrees();
                driveUtil.driveRobotDistanceForward(50, .3); //to board
               // driveUtil.driveRobotDistanceStrafeRight(15,.3);
                slides.moveToPosition(1300);
                sleep(2000);
                claw.openClaw(); //score y pixel
                slides.setCurrentState(LinearSlidesUtil.SlideState.LEVEL_ZERO);
                slides.runStateMachine();
                sleep(2000);
                driveUtil.driveRobotDistanceStrafeRight(55,.3);
                driveUtil.driveRobotDistanceForward(25,.3); //park
                sleep(2000); //reversed for red
                //add in code to do things if element is in zone 1
            }
            else if(element_zone==2){
                telemetry.addData("Zone 2 chosen", element_zone);
                telemetry.update();
                driveUtil.driveRobotDistanceForward(55,.3); //score p pixel
                slides.moveToPosition(200);
                driveUtil.driveRobotDistanceBackward(20,.3);
                driveUtil.rotateRight90Degrees();
                driveUtil.driveRobotDistanceForward(55,.3); //to board
                driveUtil.driveRobotDistanceStrafeLeft(10, .3);
                slides.moveToPosition(1300);
                sleep(2000);
                claw.openClaw(); //score y pixel
                slides.setCurrentState(LinearSlidesUtil.SlideState.LEVEL_ZERO);
                slides.runStateMachine();
                sleep(2000);
                driveUtil.driveRobotDistanceStrafeRight(50,.3);
                driveUtil.driveRobotDistanceForward(25,.3); //park
                sleep(2000); //Reversed for red
                //add in code to do things if element is in zone 2


            }
            else if (element_zone==3){
                telemetry.addData("Zone 3 chosen", element_zone);
                telemetry.update();


                driveUtil.driveRobotDistanceForward(45,.3);
                driveUtil.rotateLeft45Degrees();
                driveUtil.driveRobotDistanceForward(7  ,.3); //to spike mark
                slides.moveToPosition(200); //move slides away from pixel
                sleep(250);
                driveUtil.driveRobotDistanceBackward(15, .3);
                driveUtil.rotateRight45Degrees();
                driveUtil.driveRobotDistanceForward(62 ,.3); //to board
                driveUtil.driveRobotDistanceStrafeRight(6, .3);
                slides.moveToPosition(1300);
                sleep(2000);
                claw.openClaw(); //score y pixel
                slides.setCurrentState(LinearSlidesUtil.SlideState.LEVEL_ZERO);
                slides.runStateMachine();
                sleep(2000);
                driveUtil.driveRobotDistanceStrafeRight(40, .3);
                driveUtil.driveRobotDistanceForward(10,.3);
                sleep(2000); //reversed for red
                //add in code to do things if element is in zone 3
            }
            else {
                telemetry.addData("i'm in else", "just chillin");
            }
        } //end runopmode
    } //end of program

