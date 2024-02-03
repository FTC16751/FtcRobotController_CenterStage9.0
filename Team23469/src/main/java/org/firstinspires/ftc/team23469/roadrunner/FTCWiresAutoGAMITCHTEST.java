/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.team23469.roadrunner;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import static java.lang.Character.toLowerCase;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import org.firstinspires.ftc.team23469.robot.utilities.Learning.TeamElementSubsystem;
import org.firstinspires.ftc.team23469.robot.utilities.Production.ClawUtil;
import org.firstinspires.ftc.team23469.robot.utilities.Production.DriveUtil;
import org.firstinspires.ftc.team23469.robot.utilities.Production.LinearSlidesUtil;
import org.firstinspires.ftc.team23469.robot.utilities.Production.DriveUtil2024;

/**
 * FTC WIRES Autonomous Example for only vision detection using tensorflow and park
 */
@Autonomous(name = "Gear Girls RoadRunner Autonomous", group = "00-Autonomous", preselectTeleOp = "FTC Wires TeleOp")
public class FTCWiresAutoGAMITCHTEST extends LinearOpMode {
    /* add in our vision stuff */
    public int element_zone = 1;

    private TeamElementSubsystem teamElementDetection = null;

    boolean togglePreview = true;
    /* declare our subystems */
    private DriveUtil2024 drive = null;
    private ClawUtil claw = null;
    private LinearSlidesUtil slides = null;

    //Initializing Hardware
    public void HardwareStart() {
        telemetry.addData("Initilizing Hardware", "Start");
        telemetry.update();

        //initilize the camera utiity and vision pipeline
        teamElementDetection = new TeamElementSubsystem(hardwareMap);

/*
        //initialize the claw/wrist
        claw = new ClawUtil(hardwareMap);

        //initialize the slides
        slides = new LinearSlidesUtil(hardwareMap);
        //set things up in initiual state for autonomous (open/closed,set positions, etc)
        //example
        claw.closeClaw();
        claw.lowerWrist();

 */
    }
    //Define and declare Robot Starting Locations
    public enum ALLIANCE{
        BLUE,
        RED,
    }
    public static ALLIANCE selectedAlliance;
    public enum FIELD_SIDE {
        BACKSTAGE,
        WING,
    }
    public static FIELD_SIDE selectedFieldSide;
    public enum IDENTIFIED_SPIKE_MARK_LOCATION {
        LEFT,
        MIDDLE,
        RIGHT
    }
    public static IDENTIFIED_SPIKE_MARK_LOCATION selectedSpikeMarkLocation;
    public static IDENTIFIED_SPIKE_MARK_LOCATION identifiedSpikeMarkLocation;//= IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
    public enum AUTO_PATH {
        GATE,
        TRUSS,
    }
    public static AUTO_PATH selectedAutoPath;
    public enum PARK_LOCATION {
        INSIDE,
        OUTSIDE,
    }
    public static PARK_LOCATION selectedParkLocation;
    double waitSecondsBeforeDrop = 0;
    Boolean
            allianceselected = null,
            fieldsideselected = null,
            autopathselected = null,
            parkingselected = null,
            initilized = false,
            previousX = null,
            previousB = null,
            waitsecondsselected = false,
            spikemarklocationselected = false;



    @Override
    public void runOpMode() throws InterruptedException {

        //initilize our hardware
        HardwareStart();

        boolean previousX = false;
        boolean previousB = false;

        while (!initilized && !isStopRequested()) {
            selectAutoParams();
            //setSafeWaitSeconds();
        }

        //we added this to convert from the alliance selection string of 'BLUE' to 'blue' which is what our
        //existing opencv pipeline code expects (blue or red in lowercase).
        String curAlliance = selectedAlliance.toString().toLowerCase();

        while (!opModeIsActive() && !isStopRequested()) {
            element_zone = teamElementDetection.elementDetection(telemetry);
            if (togglePreview && gamepad2.a) {
                togglePreview = false;
                teamElementDetection.toggleAverageZone(gamepad2);
            } else if (!gamepad2.a) {
                togglePreview = true;
            }
            teamElementDetection.setAlliance(curAlliance);
            telemetry.addData("Status,","initilized");
            telemetry.addData("Alliance", selectedAlliance);
            telemetry.addData("spike mark location", selectedSpikeMarkLocation);
            telemetry.addData("Starting Field Side", selectedFieldSide);
            telemetry.addData("Path", selectedAutoPath);
            telemetry.addData("Parking Location", selectedParkLocation);
            telemetry.addData("Wait seconds before Yellow Pixel Score: ", waitSecondsBeforeDrop);
            telemetry.addLine();
            telemetry.addData("curAlliance", curAlliance);
            // Wait for the DS start button to be touched.
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();
        }

        waitForStart();


        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //convert the identified spike mark location from our existing opencv to the spike mark locations of this program
            //we used 1,2,3 wheareas this program uses Right,Left,Center
            //something to be optimized later
            updateIdentifiedSpikeMarkLocation();
            //Build parking trajectory based on last detected target by vision
            runAutonoumousMode();
        }
    }   // end runOpMode()
    private void updateIdentifiedSpikeMarkLocation() {
        //**********************BEGIN LOGGING CODE**********************************
        String name = new Object(){}.getClass().getEnclosingMethod().getName();
        StackTraceElement[] stacktrace = Thread.currentThread().getStackTrace();
        StackTraceElement e = stacktrace[3];//maybe this number needs to be corrected
        RobotLog.dd("ROBOLOG", "current method: "+name+": called from: "+e);
        //**********************END LOGGING CODE*************************************


        switch (element_zone) {
            case 1:
                identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
                break;
            case 2:
                identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                break;
            case 3:
                identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
                break;
            default:
                identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                break;
        }
    }
    public void runAutonoumousMode() {

        /**********************BEGIN LOGGING CODE**********************************
         * Logging code snippet. This block of code willl log information to android studio logger for analysis
         */
        String name = new Object(){}.getClass().getEnclosingMethod().getName();
        StackTraceElement[] stacktrace = Thread.currentThread().getStackTrace();
        StackTraceElement e = stacktrace[3];//maybe this number needs to be corrected
        RobotLog.dd("ROBOLOG", "current method: "+name+": called from: "+e);
        //**********************END LOGGING CODE*************************************


        //safeWaitSeconds(30);
        //identifiedSpikeMarkLocation = selectedSpikeMarkLocation;
        //Initialize all the separate pose paths the robot will take. these are not the path coordinates. just initialization.
        Pose2d startingPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d moveBeyondTrussPose = new Pose2d(0,0,0);  //small move to get robot past truss on start
        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0); // path to drop purple pixel on spike mark
        Pose2d midwayPose1 = new Pose2d(0,0,0); //this path can be used to move robot away from purple pixel
        Pose2d midwayPose1a = new Pose2d(0,0,0); //this path can be used as an in-between dropping purple pixel and before driving to other side
        Pose2d intakeStack = new Pose2d(0,0,0); //not used
        Pose2d midwayPose2 = new Pose2d(0,0,0); //position for driving to backdro side
        Pose2d midwayPose2a = new Pose2d(0,0,0); //safe spot on backdrop side right before scoring pixel
        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0); //positions of dropping yellow pixel
        Pose2d parkPose = new Pose2d(0,0, 0); // park position

        double slowdropYellowPixelPoseYcoordinate = 0; //positions of dropping yellow pixel

        MecanumDrive drive = new MecanumDrive(hardwareMap, startingPose); //init mecanum drive with starting position
        startingPose = new Pose2d(0, 0, Math.toRadians(0)); //Starting pose
        moveBeyondTrussPose = new Pose2d(23,0,0);

        //TODO *************REMOVE ME
        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;

        switch (selectedAlliance) {
            case BLUE:
                RobotLog.dd("ROBOLOG", "Alliance Case: " + selectedAlliance);
                switch (selectedFieldSide) {
                    case BACKSTAGE:
                        drive = new MecanumDrive(hardwareMap, startingPose);
                        // Set up poses and actions for BLUE/BACKSTAGE scenario
                        switch (identifiedSpikeMarkLocation) {
                            case LEFT:
                                RobotLog.dd("ROBOLOG", "Spike Mark : " + identifiedSpikeMarkLocation);

                                dropPurplePixelPose = new Pose2d(26, 8, Math.toRadians(45));
                                dropYellowPixelPose = new Pose2d(19, 36, Math.toRadians(90));
                                break;
                            case MIDDLE:
                                RobotLog.dd("ROBOLOG", "Spike Mark : " + identifiedSpikeMarkLocation);

                                dropPurplePixelPose = new Pose2d(34, 0, Math.toRadians(0));
                                dropYellowPixelPose = new Pose2d(27.1, 36, Math.toRadians(90));
                                break;
                            case RIGHT:
                                RobotLog.dd("ROBOLOG", "Spike Mark : " + identifiedSpikeMarkLocation);

                                dropPurplePixelPose = new Pose2d(30, -9, Math.toRadians(-45));
                                dropYellowPixelPose = new Pose2d(34, 36, Math.toRadians(90));
                                break;
                        }
                        midwayPose1 = new Pose2d(23, 0, Math.toRadians(0));
                        parkPose = new Pose2d(6, 36, Math.toRadians(90));
                        slowdropYellowPixelPoseYcoordinate = 32;
                        break;

                    case WING:
                        drive = new MecanumDrive(hardwareMap, startingPose);
                        //driveWingBlue(startingPose, dropPurplePixelPose, dropYellowPixelPose, moveAwayfromPurplePixelPose, parkPose, midwayPose1a, midwayPose2, midwayPose2a);
                        switch (identifiedSpikeMarkLocation) {
                            case LEFT:
                                dropPurplePixelPose = new Pose2d(28, 7.7, Math.toRadians(45));
                                dropYellowPixelPose = new Pose2d(21.6, 77, Math.toRadians(90));
                                break;
                            case MIDDLE:
                                dropPurplePixelPose = new Pose2d(34, 0, Math.toRadians(0));
                                dropYellowPixelPose = new Pose2d(27.1, 77, Math.toRadians(90));
                                break;
                            case RIGHT:
                                dropPurplePixelPose = new Pose2d(28, -5.6, Math.toRadians(-45));
                                dropYellowPixelPose = new Pose2d(34, 77, Math.toRadians(90));
                                break;
                        }
                        midwayPose1 = new Pose2d(18.68, -19.8, Math.toRadians(0));
                        midwayPose1a = new Pose2d(53, -17.9, Math.toRadians(-0));
                        //intakeStack = new Pose2d(52, -19,Math.toRadians(-90));
                        midwayPose2 = new Pose2d(53, 50, Math.toRadians(90));
                        midwayPose2a = new Pose2d(53, 75, Math.toRadians(90));
                        parkPose = new Pose2d(52, 82, Math.toRadians(90));
                        slowdropYellowPixelPoseYcoordinate = 86;
                        break;//break from field side
                }
                break;//break from alliance selection
            case RED:
                switch (selectedFieldSide) {
                    case BACKSTAGE:
                        RobotLog.dd("ROBOLOG", "Alliance Case: " + selectedAlliance);
                        drive = new MecanumDrive(hardwareMap, startingPose);
                        // Set up poses and actions for RED/BACKSTAGE scenario
                        switch (identifiedSpikeMarkLocation) {
                            case LEFT:
                                RobotLog.dd("ROBOLOG", "Spike Mark : " + identifiedSpikeMarkLocation);
                                dropPurplePixelPose = new Pose2d(30, 9, Math.toRadians(45));
                                dropYellowPixelPose = new Pose2d(36, -36, Math.toRadians(-90));
                                break;
                            case MIDDLE:
                                RobotLog.dd("ROBOLOG", "Spike Mark : " + identifiedSpikeMarkLocation);
                                dropPurplePixelPose = new Pose2d(34, 0, Math.toRadians(0));
                                dropYellowPixelPose = new Pose2d(29, -36, Math.toRadians(-90));
                                break;
                            case RIGHT:
                                RobotLog.dd("ROBOLOG", "Spike Mark : " + identifiedSpikeMarkLocation);
                                dropPurplePixelPose = new Pose2d(30, 0, Math.toRadians(-45));
                                dropYellowPixelPose = new Pose2d(24, -36, Math.toRadians(-90));
                                break;
                        }
                        midwayPose1 = new Pose2d(18, 0, Math.toRadians(0));
                        parkPose = new Pose2d(6, -36, Math.toRadians(-90));
                        slowdropYellowPixelPoseYcoordinate = -32;
                        break;

                    case WING:
                        drive = new MecanumDrive(hardwareMap, startingPose);
                        // Set up poses and actions for RED/WING scenario
                        switch (identifiedSpikeMarkLocation) {
                            case LEFT:
                                dropPurplePixelPose = new Pose2d(28, 8, Math.toRadians(0));
                                dropYellowPixelPose = new Pose2d(21.6, -77, Math.toRadians(90));
                                break;
                            case MIDDLE:
                                dropPurplePixelPose = new Pose2d(34, 0, Math.toRadians(0));
                                dropYellowPixelPose = new Pose2d(27.1, -77, Math.toRadians(90));
                                break;
                            case RIGHT:
                                dropPurplePixelPose = new Pose2d(28, -9, Math.toRadians(-45));
                                dropYellowPixelPose = new Pose2d(34, -77, Math.toRadians(90));
                                break;
                        }
                        midwayPose1 = new Pose2d(8, 8, Math.toRadians(0));
                        midwayPose1a = new Pose2d(18, 18, Math.toRadians(90));
                        midwayPose2 = new Pose2d(52, -62, Math.toRadians(90));
                        parkPose = new Pose2d(50, -84, Math.toRadians(90));
                        slowdropYellowPixelPoseYcoordinate = -86;
                        break; //break from field side
                }
                break; //break from alliance selection
        }

        /*********************************
         * after defining all the paths (pose2d's) for each scenario, we now execute the paths using actions.runblocking
         * mixed inbetween the actions we also execute other robot sub-system code
         *********************************/

        //Move robot to dropPurplePixel based on identified Spike Mark Location
        RobotLog.dd("ROBOLOG", "Auto Drive: Move Beyond Truss and Drop Purple Pixel ");
        RobotLog.dd("ROBOLOG", "moveBeyondTrussPose "+moveBeyondTrussPose.position+" with Heading: "+moveBeyondTrussPose.heading);
        RobotLog.dd("ROBOLOG", "dropPurplePixelPose "+dropPurplePixelPose.position+" with Heading: "+dropPurplePixelPose.heading);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveBeyondTrussPose.position, moveBeyondTrussPose.heading)
                        .strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                        .build());

        //TODO : Code to drop Purple Pixel on Spike Mark
        safeWaitSeconds(1);

        //Move robot to midwayPose1
        RobotLog.dd("ROBOLOG", "midwayPose1 "+midwayPose1.position+" with Heading: "+midwayPose1.heading);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());

        //For Blue Right and Red Left, intake pixel from stack
        if (selectedFieldSide== FIELD_SIDE.WING) {

            RobotLog.dd("ROBOLOG", "midwayPose1a "+midwayPose1a.position+" with Heading: "+midwayPose1a.heading);
            RobotLog.dd("ROBOLOG", "midwayPose2 "+midwayPose2.position+" with Heading: "+midwayPose2.heading);
            RobotLog.dd("ROBOLOG", "waitSecondsBeforeDrop "+waitSecondsBeforeDrop);
            RobotLog.dd("ROBOLOG", "midwayPose2a "+midwayPose2a.position+" with Heading: "+midwayPose2a.heading);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1a.position, midwayPose1.heading)
                            //Move robot to midwayPose2 and to dropYellowPixelPose
                            .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                            .waitSeconds(waitSecondsBeforeDrop)
                            .strafeToLinearHeading(midwayPose2a.position, midwayPose2a.heading)
                            .build());
        }


        //Move robot to midwayPose2 and to dropYellowPixelPose
        RobotLog.dd("ROBOLOG", "dropYellowPixelPose "+dropYellowPixelPose.position+" with Heading: "+dropYellowPixelPose.heading);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToLinearHeading(dropYellowPixelPose,0)
                        .build());

        //TODO : Code to drop Pixel on Backdrop
        safeWaitSeconds(1);
/*
        RobotLog.dd("ROBOLOG", "parkPose "+parkPose.position+" with Heading: "+parkPose.heading);
        //Move robot to park in Backstage
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //.strafeToLinearHeading(parkPose.position, parkPose.heading)
                        .splineToLinearHeading(parkPose,0)
                        .build());

 */
    }

    public void selectAutoParams() {

        boolean currentX = gamepad1.x;
        boolean currentB = gamepad1.b;

        if(allianceselected == null) {
            telemetry.addData("Alliance", "X = blue, B = red");
            telemetry.update();
            if (currentX && !previousX) {
                selectedAlliance = ALLIANCE.BLUE;
                allianceselected = true;
            }
            if (currentB && !previousB) {
                selectedAlliance = ALLIANCE.RED;
                allianceselected = true;
            }
        } else if (fieldsideselected == null) {
            telemetry.addData("Starting Side: ", "X = backstage, B = wing");
            telemetry.update();
            if (currentX && !previousX) {
                selectedFieldSide = FIELD_SIDE.BACKSTAGE;
                fieldsideselected = true;
            }
            if (currentB && !previousB) {
                selectedFieldSide = FIELD_SIDE.WING;
                fieldsideselected = true;
            }
        }
        /*
        else if (spikemarklocationselected == false) {
            telemetry.addData("TESTING SPIKE MARK LOCATION: ", "X = LEFT, Y = MIDDLE, B = RIGHT");
            telemetry.update();
            if (currentX && !previousX){
                selectedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
                spikemarklocationselected = true;
            }
            if (gamepad1.y) {
                selectedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                spikemarklocationselected = true;

            }
            if (currentB && !previousB){
                selectedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
                spikemarklocationselected = true;
            }
        } else if (autopathselected == null) {
            telemetry.addData("Path to Backdrop: ", "X = gate, B = truss");
            telemetry.update();
            if (currentX && !previousX) {
                selectedAutoPath = AUTO_PATH.GATE;
                autopathselected = true;
            }
            if (currentB && !previousB) {
                selectedAutoPath = AUTO_PATH.TRUSS;
                autopathselected = true;
            }
        } else if (parkingselected == null) {
            telemetry.addData("Parking location: ", "X = inside, B = outside");
            telemetry.update();
            if (currentX && !previousX) {
                selectedParkLocation = PARK_LOCATION.INSIDE;
                parkingselected = true;
            }
            if (currentB && !previousB) {
                selectedParkLocation = PARK_LOCATION.OUTSIDE;
                parkingselected = true;
            }
        }
        */
        else if (waitsecondsselected == false) {
            telemetry.addData("---------------------------------------","");
            telemetry.addData("set safe wait using YA on Logitech on gamepad 1:","");
            telemetry.addData("    2 second wait   ", "(dpad left)");
            telemetry.addData("    5 second wait ", "(dpad up)");
            telemetry.addData("    7 second wait    ", "(dpad right)");
            telemetry.addData("    10 second wait  ", "(dpad down)");
            telemetry.update();
            if(currentX && !previousX){
                waitSecondsBeforeDrop = 2;
                waitsecondsselected = true;
            }
            if(gamepad1.y){
                waitSecondsBeforeDrop = 5;
                waitsecondsselected = true;
            }
            if(currentB && !previousB){
                waitSecondsBeforeDrop = 7;
                waitsecondsselected = true;
            }
            if(gamepad1.a){
                waitSecondsBeforeDrop = 10;
                waitsecondsselected = true;
            }
        }
        else {
            initilized = true;
        }
        previousX = currentX;
        previousB = currentB;
    }
    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }
}   // end class