package org.firstinspires.ftc.team23469.Autonomous.InUse;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team23469.robot.utilities.Learning.TeamElementSubsystem;
import org.firstinspires.ftc.team23469.robot.utilities.Production.ClawUtil;
import org.firstinspires.ftc.team23469.robot.utilities.Production.DriveUtil;
import org.firstinspires.ftc.team23469.robot.utilities.Production.LinearSlidesUtil;

@Autonomous(name="Red Vision Backstage", group="Auto")

public class AutoRedVisionBackstage extends LinearOpMode {
    public int element_zone = 1;
    private TeamElementSubsystem teamElementDetection = null;
    boolean togglePreview = true;
    private DriveUtil drive = null;
    private ClawUtil claw = null;
    private LinearSlidesUtil slides = null;

//Initializing Hardware
    public void HardwareStart() {
        telemetry.addData("Initilizing Hardware", "Start");
        telemetry.update();

        //initilize the camera utiity and vision pipeline
        teamElementDetection = new TeamElementSubsystem(hardwareMap);

        //initilize the drive train
        drive = new DriveUtil(this);
        drive.init(hardwareMap);

        //initialize the claw/wrist
        claw = new ClawUtil(hardwareMap);

        //initialize the slides
        slides = new LinearSlidesUtil(hardwareMap);

        //set things up in initiual state for autonomous (open/closed,set positions, etc)
        //example
        claw.closeClaw();
        claw.lowerWrist();
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
                slides.moveToPosition(200);
                drive.driveRobotDistanceForward(50, .3); //score p pixel
                drive.rotateLeft90Degrees();
                drive.driveRobotDistanceForward(16,.3);
                drive.driveRobotDistanceBackward(7,.3);
                while (drive.motorisBusyLF()) {
                }
                claw.openRightClaw();
                sleep(500);
                drive.driveRobotDistanceBackward(20,.3);
                drive.rotateRight90Degrees();
                drive.rotateRight90Degrees();
                drive.driveRobotDistanceForward(50, .3); //to board
               // driveUtil.driveRobotDistanceStrafeRight(15,.3);
                slides.moveToPosition(1300);
                sleep(2000);
                claw.raiseWrist();
                drive.driveRobotDistanceForward(6.5,.15); //to board slowly
                while (drive.motorisBusyLF()){}
                claw.openClaw(); //score y pixel
                sleep(1000);
                drive.driveRobotDistanceBackward(6,.5); //move away from board
                while (drive.motorisBusyLF()){}
                claw.closeClaw();
                claw.lowerWrist();
                sleep(250);
                slides.setCurrentState(LinearSlidesUtil.SlideState.LEVEL_ZERO);
                slides.runStateMachine();
                drive.driveRobotDistanceStrafeRight(55,.3);
                drive.driveRobotDistanceForward(25,.3); //park
                sleep(2000); //reversed for red
                //add in code to do things if element is in zone 1
            }
            else if(element_zone==2){
                telemetry.addData("Zone 2 chosen", element_zone);
                telemetry.update();
                slides.moveToPosition(200);
                drive.driveRobotDistanceForward(60,.3); //score p pixel
                drive.driveRobotDistanceBackward(5,.3);
                while (drive.motorisBusyLF()) {
                }
                claw.openRightClaw();
                sleep(500);
                drive.driveRobotDistanceBackward(20,.3);
                while (drive.motorisBusyLF()){}
                claw.closeRightClaw();
                drive.rotateRight90Degrees();
                drive.driveRobotDistanceForward(58,.3); //to board
                drive.driveRobotDistanceStrafeLeft(10, .3);
                slides.moveToPosition(1300);
                sleep(2000);
                claw.raiseWrist();
                drive.driveRobotDistanceForward(5.5,.15); //to board slowly
                while (drive.motorisBusyLF()){}
                claw.openClaw(); //score y pixel
                sleep(1000);
                drive.driveRobotDistanceBackward(6,.5); //move away from board
                while (drive.motorisBusyLF()){}
                claw.closeClaw();
                claw.lowerWrist();
                sleep(250);
                slides.setCurrentState(LinearSlidesUtil.SlideState.LEVEL_ZERO);
                slides.runStateMachine();
                drive.driveRobotDistanceStrafeRight(45,.3);
                drive.driveRobotDistanceForward(20,.3); //park
                sleep(2000); //Reversed for red
            }
            else if (element_zone==3){
                telemetry.addData("Zone 3 chosen", element_zone);
                telemetry.update();
                slides.moveToPosition(200); //move slides to travel
                drive.driveRobotDistanceForward(45,.3);
                drive.rotateRight45Degrees();
                drive.driveRobotDistanceForward(15  ,.3); //to spike mark
                drive.driveRobotDistanceBackward(5  ,.3); //to spike mark
                while (drive.motorisBusyLF()) {}
                claw.openRightClaw();
                sleep(250);
                drive.driveRobotDistanceBackward(15, .3);
                while (drive.motorisBusyLF()) {}
                claw.closeRightClaw();
                drive.rotateRight45Degrees();
                drive.driveRobotDistanceForward(55 ,.3); //to board
                drive.driveRobotDistanceStrafeRight(6, .3);
                slides.moveToPosition(1300);
                sleep(2000);
                claw.raiseWrist();
                drive.driveRobotDistanceForward(8,.15); //to board slowly
                while (drive.motorisBusyLF()){}
                claw.openClaw(); //score y pixel
                sleep(1000);
                drive.driveRobotDistanceBackward(5.5,.5); //move away from board
                while (drive.motorisBusyLF()){}
                claw.closeClaw();
                claw.lowerWrist();
                sleep(250);
                slides.setCurrentState(LinearSlidesUtil.SlideState.LEVEL_ZERO);
                slides.runStateMachine();
                drive.driveRobotDistanceStrafeRight(40, .3);
                drive.driveRobotDistanceForward(10,.3);
                sleep(2000); //reversed for red
                //add in code to do things if element is in zone 3
            }
            else {
                telemetry.addData("i'm in else", "just chillin");
            }
        } //end runopmode
    } //end of program

