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
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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

/**
 * FTC WIRES Autonomous Example for only vision detection using tensorflow and park
 */
@Autonomous(name = "FTC Wires testing around", group = "00-Autonomous", preselectTeleOp = "FTC Wires TeleOp")
public class FTCWiresAutoGAMITCHTEST extends LinearOpMode {
    public static String TEAM_NAME = "EDIT TEAM NAME"; //TODO: Enter team Name
    public static int TEAM_NUMBER = 0; //TODO: Enter team Number

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    //Vision parameters
    private FTCWiresAutoVisionOpenCV.VisionOpenCV visionOpenCV;
    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;

    public enum IDENTIFIED_SPIKE_MARK_LOCATION {
        LEFT,
        MIDDLE,
        RIGHT
    }
    public static IDENTIFIED_SPIKE_MARK_LOCATION selectedSpikeMarkLocation;
    public enum ALLIANCE{
        BLUE,
        RED,
    } ALLIANCE selectedAlliance;
    public enum FIELD_SIDE {
        BACKSTAGE,
        WING,
    } FIELD_SIDE selectedFieldSide;
    public enum AUTO_PATH {
        GATE,
        TRUSS,
    } AUTO_PATH selectedAutoPath;
    public enum PARK_LOCATION {
        INSIDE,
        OUTSIDE,
    } PARK_LOCATION selectedParkLocation;
    public static IDENTIFIED_SPIKE_MARK_LOCATION identifiedSpikeMarkLocation;//= IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
    double waitSecondsBeforeDrop = 0;

    Boolean allianceselected = null;
    Boolean fieldsideselected = null;
    Boolean autopathselected = null;
    Boolean parkingselected = null;
    boolean initilized = false;
    Boolean previousX = null;
    Boolean previousB = null;
    Boolean waitsecondsselected = false;
    Boolean spikemarklocationselected = false;

    /* add in our vision stuff */
    private TeamElementSubsystem teamElementDetection = null;
    boolean togglePreview = true;
    public int element_zone = 1;

    /* declare our subystems */
    private DriveUtil drive = null;
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

        //Key Pay inputs to selecting Starting Position of robot
        //selectStartingPosition();
        telemetry.addData("Status,","initilized");
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("spike mark location", selectedSpikeMarkLocation);

        telemetry.addData("Starting Field Side", selectedFieldSide);
        telemetry.addData("Path", selectedAutoPath);
        telemetry.addData("Parking Location", selectedParkLocation);
        telemetry.addData("Wait seconds before Yellow Pixel Score: ", waitSecondsBeforeDrop);
        telemetry.addLine();


        String curAlliance = selectedAlliance.toString().toLowerCase();
        telemetry.addData("curAlliance", curAlliance);
        telemetry.update();
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
            if (element_zone == 1) {
                identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
            } else if (element_zone == 2) {
                identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
            } else if (element_zone == 3) {
                identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
            } else {
                //default to middle if nothing was detected
                identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
            }
            //Build parking trajectory based on last detected target by vision
            runAutonoumousMode();
        }
    }   // end runOpMode()

    public void runAutonoumousMode() {
        telemetry.addData("wait seconds boolean:", waitSecondsBeforeDrop);
        telemetry.addData("wait seconds boolean:", waitsecondsselected);
        telemetry.update();
        //safeWaitSeconds(30);
        //identifiedSpikeMarkLocation = selectedSpikeMarkLocation;
        //Initialize all the separate pose paths the robot will take. these are not the path coordinates. just initialization.
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d moveBeyondTrussPose = new Pose2d(0,0,0);  //small move to get robot past truss on start
        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0); // path to drop purple pixel on spike mark
        Pose2d midwayPose1 = new Pose2d(0,0,0); //this path can be used to move robot away from purple pixel
        Pose2d midwayPose1a = new Pose2d(0,0,0); //this path can be used as an in-between dropping purple pixel and before driving to other side
        Pose2d intakeStack = new Pose2d(0,0,0); //not used
        Pose2d midwayPose2 = new Pose2d(0,0,0); //position for driving to backdro side
        Pose2d midwayPose2a = new Pose2d(0,0,0); //safe spot on backdrop side right before scoring pixel
        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0); //positions of dropping yellow pixel
        Pose2d parkPose = new Pose2d(0,0, 0); // park position
        //waitSecondsBeforeDrop = 0;

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose); //init mecanum drive with starting position

        initPose = new Pose2d(0, 0, Math.toRadians(0)); //Starting pose
        moveBeyondTrussPose = new Pose2d(23,0,0);
        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
        switch (selectedAlliance) {
            case BLUE:
                if (selectedFieldSide == FIELD_SIDE.BACKSTAGE) {
                    drive = new MecanumDrive(hardwareMap, initPose);
                    switch(identifiedSpikeMarkLocation){
                        case LEFT:
                            dropPurplePixelPose = new Pose2d(26, 8, Math.toRadians(45));
                            dropYellowPixelPose = new Pose2d(21.6, 0, Math.toRadians(90));
                            break;
                        case MIDDLE:
                            dropPurplePixelPose = new Pose2d(34, 0, Math.toRadians(0));
                            dropYellowPixelPose = new Pose2d(27.1, 0,  Math.toRadians(90));
                            break;
                        case RIGHT:
                            dropPurplePixelPose = new Pose2d(30, -9, Math.toRadians(-45));
                            dropYellowPixelPose = new Pose2d(34, 0, Math.toRadians(90));
                            break;
                    }
                    midwayPose1 = new Pose2d(23, 0, Math.toRadians(0));
                    //waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board
                    parkPose = new Pose2d(8, 30, Math.toRadians(90));
                }
                else if (selectedFieldSide == FIELD_SIDE.WING) {
                        drive = new MecanumDrive(hardwareMap, initPose);
                        switch(identifiedSpikeMarkLocation){
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

                        //waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                        parkPose = new Pose2d(52, 82, Math.toRadians(90));

                }
                break;
            case RED:
                if (selectedFieldSide == FIELD_SIDE.BACKSTAGE) {
                    drive = new MecanumDrive(hardwareMap, initPose);
                    switch (identifiedSpikeMarkLocation) {
                        case LEFT:
                            dropPurplePixelPose = new Pose2d(28, 9, Math.toRadians(45));
                            dropYellowPixelPose = new Pose2d(21, -36, Math.toRadians(90));
                            break;
                        case MIDDLE:
                            dropPurplePixelPose = new Pose2d(34, 0, Math.toRadians(0));
                            dropYellowPixelPose = new Pose2d(29, -36, Math.toRadians(90));
                            break;
                        case RIGHT:
                            dropPurplePixelPose = new Pose2d(28, -8, Math.toRadians(-45));
                            dropYellowPixelPose = new Pose2d(37, -36, Math.toRadians(90));
                            break;
                    }
                    midwayPose1 = new Pose2d(14, -13, Math.toRadians(45));
                    //waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                    parkPose = new Pose2d(8, -30, Math.toRadians(90));
                }
                else if (selectedFieldSide == FIELD_SIDE.WING) {
                    drive = new MecanumDrive(hardwareMap, initPose);
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
                    intakeStack = new Pose2d(52, 19, Math.toRadians(90));
                    midwayPose2 = new Pose2d(52, -62, Math.toRadians(90));
                    //waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                    parkPose = new Pose2d(50, -84, Math.toRadians(90));
                }
                break;
        }

        //Move robot to dropPurplePixel based on identified Spike Mark Location
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveBeyondTrussPose.position, moveBeyondTrussPose.heading)
                        .strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                        .build());

        //TODO : Code to drop Purple Pixel on Spike Mark
        safeWaitSeconds(1);

        //Move robot to midwayPose1
        telemetry.addData("midwayPose", "start");
        telemetry.update();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());

        telemetry.addData("midwayPose1", "start");
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1a.position, midwayPose1.heading)
                        .build());

        //For Blue Right and Red Left, intake pixel from stack
        if (selectedFieldSide== FIELD_SIDE.WING) {
            telemetry.addData("midwayPose2", "start");
            telemetry.update();
            //Move robot to midwayPose2 and to dropYellowPixelPose
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                            .build());

            safeWaitSeconds(waitSecondsBeforeDrop);

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose2a.position, midwayPose2a.heading)
                            .build());
        }

        telemetry.addData("dropYellowPixelPose", "start");
        telemetry.update();
        //Move robot to midwayPose2 and to dropYellowPixelPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToLinearHeading(dropYellowPixelPose,0)
                        .build());

        //TODO : Code to drop Pixel on Backdrop
        safeWaitSeconds(1);

        telemetry.addData("park", "start");
        telemetry.update();
        //Move robot to park in Backstage
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        //.splineToLinearHeading(parkPose,0)
                        .build());
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
        } else if (spikemarklocationselected == false) {
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
        } else if (waitsecondsselected == false) {
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

    /**
     * Initialize the Open CV Object Detection processor.
     */
    public Rect rectLeftOfCameraMid, rectRightOfCameraMid;
    private void initOpenCV() {
       // visionOpenCV = new VisionOpenCV(hardwareMap);

        if (startPosition == START_POSITION.RED_LEFT ||
                startPosition == START_POSITION.BLUE_LEFT) {
            rectLeftOfCameraMid = new Rect(10, 40, 150, 240);
            rectRightOfCameraMid = new Rect(160, 40, 470, 160);
        } else { //RED_RIGHT or BLUE_RIGHT
            rectLeftOfCameraMid = new Rect(10, 40, 470, 160);
            rectRightOfCameraMid = new Rect(480, 40, 150, 240);
        }
    }

    /**
     * Add telemetry about Object Detection recognitions.
     */
    private void runOpenCVObjectDetection() {
        //visionOpenCV.getSelection();
        telemetry.addLine("Open CV based Vision Processor for Team Element Detection");
        telemetry.addData("Identified Parking Location", identifiedSpikeMarkLocation);
      //  telemetry.addData("SatLeftOfCameraMid", visionOpenCV.satRectLeftOfCameraMid);

       // telemetry.addData("SatRightOfCameraMid", visionOpenCV.satRectRightOfCameraMid);
       // telemetry.addData("SatRectNone", visionOpenCV.satRectNone);
        telemetry.update();
    }

    public class VisionOpenCV implements VisionProcessor {

        CameraSelectedAroundMid selectionAroundMid = CameraSelectedAroundMid.NONE;

        public VisionPortal visionPortal;

        Mat submat = new Mat();
        Mat hsvMat = new Mat();

        public double satRectLeftOfCameraMid, satRectRightOfCameraMid;
        public double satRectNone = 40.0;

        public VisionOpenCV(HardwareMap hardwareMap){
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), this);
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

            satRectLeftOfCameraMid = getAvgSaturation(hsvMat, rectLeftOfCameraMid);
            satRectRightOfCameraMid = getAvgSaturation(hsvMat, rectRightOfCameraMid);

            if ((satRectLeftOfCameraMid > satRectRightOfCameraMid) && (satRectLeftOfCameraMid > satRectNone)) {
                return CameraSelectedAroundMid.LEFT_OF_CAMERA_MID;
            } else if ((satRectRightOfCameraMid > satRectLeftOfCameraMid) && (satRectRightOfCameraMid > satRectNone)) {
                return CameraSelectedAroundMid.RIGHT_OF_CAMERA_MID;
            }
            return CameraSelectedAroundMid.NONE;
        }

        protected double getAvgSaturation(Mat input, Rect rect) {
            submat = input.submat(rect);
            Scalar color = Core.mean(submat);
            return color.val[1];
        }

        private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
            int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
            int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
            int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
            int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

            return new android.graphics.Rect(left, top, right, bottom);
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            Paint selectedPaint = new Paint();
            selectedPaint.setColor(Color.RED);
            selectedPaint.setStyle(Paint.Style.STROKE);
            selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

            Paint nonSelectedPaint = new Paint(selectedPaint);
            nonSelectedPaint.setColor(Color.GREEN);

            android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeftOfCameraMid, scaleBmpPxToCanvasPx);
            android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectRightOfCameraMid, scaleBmpPxToCanvasPx);

            selectionAroundMid = (CameraSelectedAroundMid) userContext;
            switch (selectionAroundMid) {
                case LEFT_OF_CAMERA_MID:
                    canvas.drawRect(drawRectangleLeft, selectedPaint);
                    canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                    break;
                case RIGHT_OF_CAMERA_MID:
                    canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                    canvas.drawRect(drawRectangleMiddle, selectedPaint);
                    break;
                case NONE:
                    canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                    canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                    break;
            }
        }

        public void getSelection() {
            if (startPosition == START_POSITION.RED_LEFT ||
                    startPosition == START_POSITION.BLUE_LEFT) {
                switch (selectionAroundMid) {
                    case NONE:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
                        break;
                    case LEFT_OF_CAMERA_MID:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
                        break;
                    case RIGHT_OF_CAMERA_MID:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                        break;
                }
            } else { //RED_RIGHT or BLUE_RIGHT
                switch (selectionAroundMid) {
                    case NONE:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
                        break;
                    case LEFT_OF_CAMERA_MID:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                        break;
                    case RIGHT_OF_CAMERA_MID:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
                        break;
                }
            }
        }
    }

    public enum CameraSelectedAroundMid {
        NONE,
        LEFT_OF_CAMERA_MID,
        RIGHT_OF_CAMERA_MID
    }
}   // end class