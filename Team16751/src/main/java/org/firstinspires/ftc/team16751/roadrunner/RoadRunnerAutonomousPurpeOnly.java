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

package org.firstinspires.ftc.team16751.roadrunner;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.team16751.robot.utilities.ArmUtil;
import org.firstinspires.ftc.team16751.robot.utilities.ClawUtil;
import org.firstinspires.ftc.team16751.robot.utilities.NewDriveUtil2024;
import org.firstinspires.ftc.team16751.robot.utilities.TeamElementSubsystem;


@Autonomous(name = "P3 Road Runner Autonomous Purple only", group = "00-Autonomous", preselectTeleOp = "Driver Control Center Stage")
@Disabled
public class RoadRunnerAutonomousPurpeOnly extends LinearOpMode {
    /* add in our vision stuff */
    public int element_zone = 1;

    private TeamElementSubsystem teamElementDetection = null;

    boolean togglePreview = true;
    private NewDriveUtil2024 P3drive = null;
    ArmUtil arm = new ArmUtil(this);
    ClawUtil claw = new ClawUtil(this);
    //Initializing Hardware
    public void HardwareStart() {
        teamElementDetection = new TeamElementSubsystem(hardwareMap);
        P3drive = new NewDriveUtil2024(this);
        P3drive.init(hardwareMap,telemetry);
        P3drive.resetIMUYaw();
        arm.init(hardwareMap);
        claw.init(hardwareMap);
        claw.setClawClosed();
        arm.setWristPosition(.1);
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
            allianceselected = null, fieldsideselected = null, autopathselected = null,
            parkingselected = null, initilized = false, previousX = null, previousB = null,
            waitsecondsselected = false, spikemarklocationselected = false;
    long startTime;
    long timeoutMillis =5000;
    String curAlliance;
    //@Override
    public void runOpMode() {//throws InterruptedException {

        //initilize our hardware
        HardwareStart();

        boolean previousX = false;
        boolean previousB = false;

        while (!initilized && !isStopRequested()) {
            selectAutoParams();
        }

        //we added this to convert from the alliance selection string of 'BLUE' to 'blue' which is what our
        //existing opencv pipeline code expects (blue or red in lowercase).
        curAlliance = selectedAlliance.toString().toLowerCase();

        //loop through the following while opmode is NOT active:
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
        arm.setWristPosition(.45);
        long startTime = System.currentTimeMillis();

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
        while ((System.currentTimeMillis() - startTime) < timeoutMillis) {
            element_zone = teamElementDetection.elementDetection(telemetry);
            if (togglePreview && gamepad2.a) {
                togglePreview = false;
                teamElementDetection.toggleAverageZone(gamepad2);
            } else if (!gamepad2.a) {
                togglePreview = true;
            }
            teamElementDetection.setAlliance(curAlliance);
        }
        String name = new Object(){}.getClass().getEnclosingMethod().getName();
        StackTraceElement[] stacktrace = Thread.currentThread().getStackTrace();
        StackTraceElement e = stacktrace[3];//maybe this number needs to be corrected
        RobotLog.dd("ROBOLOG", "current method: "+name+": called from: "+e);

        //Initialize all the separate pose paths the robot will take. these are not the path coordinates. just initialization.
        Pose2d startingPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d moveBeyondTrussPose = new Pose2d(0,0,0);  //small move to get robot past truss on start
        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0); // path to drop purple pixel on spike mark
        Pose2d moveAwayfromPurplePixelPose = new Pose2d(0,0,0); //this path can be used to move robot away from purple pixel
        Pose2d midwayPose1a = new Pose2d(0,0,0); //this path can be used as an in-between dropping purple pixel and before driving to other side
        Pose2d intakeStack = new Pose2d(0,0,0); //not used
        Pose2d midwayPose2 = new Pose2d(0,0,0); //position for driving to backdro side
        Pose2d midwayPose2a = new Pose2d(0,0,0); //safe spot on backdrop side right before scoring pixel
        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0); //positions of dropping yellow pixel
        Pose2d parkPose = new Pose2d(0,0, 0); // park position

        double slowdropYellowPixelPoseYcoordinate = 0; //positions of dropping yellow pixel
        double safeMoveFromBackDrop = 0; //positions go after dropping yellow pixel

        MecanumDrive drive = new MecanumDrive(hardwareMap, startingPose); //init mecanum drive with starting position
        startingPose = new Pose2d(0, 0, Math.toRadians(0)); //Starting pose
        moveBeyondTrussPose = new Pose2d(15,0,0);

        switch (selectedAlliance) {
            case BLUE:
                RobotLog.dd("ROBOLOG", "Alliance Case: "+selectedAlliance);
                switch (selectedFieldSide) {
                    case BACKSTAGE:
                        drive = new MecanumDrive(hardwareMap, startingPose);
                        //driveBackstageBlue(startingPose,dropPurplePixelPose, dropYellowPixelPose, moveAwayfromPurplePixelPose, parkPose);
                        // Set up poses and actions for BLUE/BACKSTAGE scenario
                        switch(identifiedSpikeMarkLocation){
                            case LEFT:
                                dropPurplePixelPose = new Pose2d(21, 17, Math.toRadians(0));
                                dropYellowPixelPose = new Pose2d(23, 30, Math.toRadians(-90));
                                break;
                            case MIDDLE:
                                dropPurplePixelPose = new Pose2d(30, 0, Math.toRadians(0));
                                dropYellowPixelPose = new Pose2d(30, 30,  Math.toRadians(-90));
                                break;
                            case RIGHT:
                                dropPurplePixelPose = new Pose2d(24, -6, Math.toRadians(-45));
                                dropYellowPixelPose = new Pose2d(37, 25, Math.toRadians(-90));
                                break;
                        }
                        moveAwayfromPurplePixelPose = new Pose2d(14, 0, Math.toRadians(0));

                        switch(selectedParkLocation){
                            case OUTSIDE:
                                parkPose = new Pose2d(6, 37, Math.toRadians(-90));
                                break;
                            case INSIDE:
                                parkPose = new Pose2d(52, 37, Math.toRadians(-90));
                                break;
                        }
                        slowdropYellowPixelPoseYcoordinate = 42;
                        safeMoveFromBackDrop = 30;
                        break;//break from field side

                    case WING:
                        drive = new MecanumDrive(hardwareMap, startingPose);
                        //driveWingBlue(startingPose, dropPurplePixelPose, dropYellowPixelPose, moveAwayfromPurplePixelPose, parkPose, midwayPose1a, midwayPose2, midwayPose2a);
                        switch(identifiedSpikeMarkLocation){
                            case LEFT:
                                dropPurplePixelPose = new Pose2d(22, 3.5, Math.toRadians(45));
                                dropYellowPixelPose = new Pose2d(25, 79, Math.toRadians(-90));
                                break;
                            case MIDDLE:
                                dropPurplePixelPose = new Pose2d(30, 0, Math.toRadians(0));
                                dropYellowPixelPose = new Pose2d(28, 79, Math.toRadians(-90));
                                break;
                            case RIGHT:
                                dropPurplePixelPose = new Pose2d(22, -3.5, Math.toRadians(-30));
                                dropYellowPixelPose = new Pose2d(35, 79, Math.toRadians(-90));
                                break;
                        }
                      switch(selectedAutoPath){
                          case GATE:
                              moveAwayfromPurplePixelPose = new Pose2d(14, -23, Math.toRadians(0));
                              midwayPose1a = new Pose2d(60, -23, Math.toRadians(0));
                              midwayPose2 = new Pose2d(53, 60, Math.toRadians(-90));
                              break;//break from field side
                          case TRUSS:
                              moveAwayfromPurplePixelPose = new Pose2d(5.5, -10, Math.toRadians(-90));
                              midwayPose1a = new Pose2d(5.5, 40, Math.toRadians(-90));
                              midwayPose2 = new Pose2d(5.5, 65, Math.toRadians(-90));
                              break;//break from field side
                      }
                      switch(selectedParkLocation){
                          case OUTSIDE:
                              parkPose = new Pose2d(6, 88, Math.toRadians(-90));
                              break;
                          case INSIDE:
                              parkPose = new Pose2d(52, 88, Math.toRadians(-90));
                              break;

                      }
                        slowdropYellowPixelPoseYcoordinate = 88;
                        safeMoveFromBackDrop = 79;
                        break;
                }
                break;//break from alliance selection
            case RED:
                switch (selectedFieldSide) {
                    case BACKSTAGE:
                        drive = new MecanumDrive(hardwareMap, startingPose);
                        //driveBackstageRed(startingPose, dropPurplePixelPose, dropYellowPixelPose, moveAwayfromPurplePixelPose, parkPose);
                        // Set up poses and actions for RED/BACKSTAGE scenario
                        switch (identifiedSpikeMarkLocation) {
                            case LEFT:
                                dropPurplePixelPose = new Pose2d(28, 9, Math.toRadians(45));
                                dropYellowPixelPose = new Pose2d(32, -35, Math.toRadians(90));
                                break;
                            case MIDDLE:
                                dropPurplePixelPose = new Pose2d(30, 0, Math.toRadians(0));
                                dropYellowPixelPose = new Pose2d(23, -35, Math.toRadians(90));
                                break;
                            case RIGHT:
                                dropPurplePixelPose = new Pose2d(28, -8, Math.toRadians(-45));
                                dropYellowPixelPose = new Pose2d(20, -35, Math.toRadians(90));
                                break;
                        }
                        switch(selectedParkLocation){
                            case OUTSIDE:
                                parkPose = new Pose2d(8, -30, Math.toRadians(90));
                                break;
                            case INSIDE:
                                parkPose = new Pose2d(52, -30, Math.toRadians(90));


                        }
                        moveAwayfromPurplePixelPose = new Pose2d(14, 0, Math.toRadians(45));
                        slowdropYellowPixelPoseYcoordinate = -32;
                        safeMoveFromBackDrop = -35;
                     break;

                    case WING:

                        drive = new MecanumDrive(hardwareMap, startingPose);
                        //driveWingRed(startingPose, dropPurplePixelPose, dropYellowPixelPose, moveAwayfromPurplePixelPose, parkPose, midwayPose1a, midwayPose2, midwayPose2a);
                        // Set up poses and actions for RED/WING scenario
                        switch (identifiedSpikeMarkLocation) {
                            case LEFT:
                                dropPurplePixelPose = new Pose2d(20, 9, Math.toRadians(0));
                                dropYellowPixelPose = new Pose2d(32, -77, Math.toRadians(90));
                                break;
                            case MIDDLE:
                                dropPurplePixelPose = new Pose2d(30, 0, Math.toRadians(0));
                                dropYellowPixelPose = new Pose2d(29, -77, Math.toRadians(90));
                                break;
                            case RIGHT:
                                dropPurplePixelPose = new Pose2d(23, 0, Math.toRadians(-45));
                                dropYellowPixelPose = new Pose2d(22, -77, Math.toRadians(90));
                                break;
                        }
                      switch (selectedAutoPath){
                          case GATE:
                              moveAwayfromPurplePixelPose = new Pose2d(14, 23, Math.toRadians(0));
                              //define those paths specific to wing
                              midwayPose2 = new Pose2d(55, -41, Math.toRadians(90));
                              break;
                          case TRUSS:
                              moveAwayfromPurplePixelPose = new Pose2d(5, 0, Math.toRadians(90));
                              //define those paths specific to wing
                              midwayPose2 = new Pose2d(6, -41, Math.toRadians(90));
                              break;
                      }
               switch(selectedParkLocation){
                   case INSIDE:
                       parkPose = new Pose2d(8, -77, Math.toRadians(90));
                               break;
                   case OUTSIDE:
                       parkPose = new Pose2d(52, -77, Math.toRadians(90));
                        break;
               }

                        slowdropYellowPixelPoseYcoordinate = -86;
                        safeMoveFromBackDrop = -77;
                    break; //break from field side
                }
                break; //break from alliance selection
        }

        /*********************************
         * after defining all the paths (pose2d's) for each scenario, we now execute the paths using actions.runblocking
         * mixed inbetween the actions we also execute other robot sub-system code
         *********************************/

        // Move robot to dropPurplePixel based on identified Spike Mark Location
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveBeyondTrussPose.position, moveBeyondTrussPose.heading)
                        .strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                        .build());

        //drop Purple Pixel on Spike Mark
        claw.openRightHand();
        safeWaitSeconds(.75);
        arm.setWristPosition(0.45);
        arm.raiseToPositionNoPID(2,.5,false); //low score
        //safeWaitSeconds(.5);
        //arm.setWristPosition(0.0);
        //claw.closeRightHand();

        //move robot back to a safe position (all scenarios)
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveAwayfromPurplePixelPose.position, moveAwayfromPurplePixelPose.heading)
                        .build());


        //set arm back to init position
        arm.setWristPosition(0.45);
        claw.setClawClosed();
        arm.raiseToPositionNoPID(1,.5,false); //init

        //we are done end of auto code
    }


    // New helper methods to reduce redundancy
    private void driveBackstageBlue(Pose2d initPose, Pose2d dropPurplePixelPose, Pose2d dropYellowPixelPose,Pose2d moveAwayfromPurplePixelPose, Pose2d parkPose) {

    }

    private void driveWingBlue(Pose2d initPose, Pose2d dropPurplePixelPose, Pose2d dropYellowPixelPose,Pose2d moveAwayfromPurplePixelPose, Pose2d parkPose, Pose2d midwayPose1a,Pose2d midwayPose2, Pose2d midwayPose2a) {
    }

    private void driveBackstageRed(Pose2d initPose, Pose2d dropPurplePixelPose, Pose2d dropYellowPixelPose,Pose2d moveAwayfromPurplePixelPose, Pose2d parkPose) {
    }

    private void driveWingRed(Pose2d initPose, Pose2d dropPurplePixelPose, Pose2d dropYellowPixelPose,Pose2d moveAwayfromPurplePixelPose, Pose2d parkPose, Pose2d midwayPose1a,Pose2d midwayPose2, Pose2d midwayPose2a) {
    }

    //Initializing Hardware
 /*   public void HardwareStart() {

        arm.init(hardwareMap);
        claw.init(hardwareMap);
        //initilize the camera utiity and vision pipeline
        teamElementDetection = new TeamElementSubsystem(hardwareMap);
        claw.setClawClosed();
        arm.setWristPosition(.45);

        //initialize the claw/wrist
        claw = new ClawUtil(hardwareMap);

        //initialize the slides
        slides = new LinearSlidesUtil(hardwareMap);

        //set things up in initiual state for autonomous (open/closed,set positions, etc)
        //example
        claw.closeClaw();
        claw.lowerWrist();


    }
    */
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
        else if (autopathselected == null && selectedFieldSide == FIELD_SIDE.WING) {
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
        else if (waitsecondsselected == false && selectedFieldSide == FIELD_SIDE.WING) {
            telemetry.addData("---------------------------------------","");
            telemetry.addData("set safe wait using YA on Logitech on gamepad 1:","");
            telemetry.addData("    2 second wait   ", "(x)");
            telemetry.addData("    5 second wait ", "(y)");
            telemetry.addData("    7 second wait    ", "(b)");
            telemetry.addData("    10 second wait  ", "(a)");
            telemetry.update();
            if(currentX && !previousX){
                waitSecondsBeforeDrop = 0;
                waitsecondsselected = true;
            }
            if(gamepad1.y){
                waitSecondsBeforeDrop = 2;
                waitsecondsselected = true;
            }
            if(currentB && !previousB){
                waitSecondsBeforeDrop = 5;
                waitsecondsselected = true;
            }
            if(gamepad1.a){
                waitSecondsBeforeDrop = 7;
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