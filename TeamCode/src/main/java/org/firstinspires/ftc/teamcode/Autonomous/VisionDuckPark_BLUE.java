package org.firstinspires.ftc.teamcode.Autonomous;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot_utilities.DriveUtil2023;
import org.firstinspires.ftc.teamcode.robot_utilities.OpenCVUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This is a simple routine to test drive utility class.
 */
@Autonomous(name="Vision Duck and Park BLUE", group="Blue Autonomous")
public class VisionDuckPark_BLUE extends LinearOpMode {

    //instantiate the sub-systems (drive, spinner, etc)
    DriveUtil2023 drive = new DriveUtil2023(this);
    OpenCVUtil vision = new OpenCVUtil();

    //open the camera opencv image pipeline (analysis)
    private OpenCVUtil.SkystoneDeterminationPipeline pipeline;

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {

        //initialize the camera
        initVision();

        //initialize the hardware mapping
        drive.init(hardwareMap);

        //wait a few seconds for the robot and camera to initialize
        sleep(2000);

        waitForStart();

        if (isStopRequested()) return;

        // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

            //Display what position camera saw the duck/tse on the barcode
            String duckPosition = String.valueOf(pipeline.getAnalysis());
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Duck Position", duckPosition);
            telemetry.update();

            //depending on which barcode the duck/tse was found, set the right arm position height for delivery
            if (duckPosition == "CENTER") {

            } else if (duckPosition == "LEFT" ) {

            } else if (duckPosition == "RIGHT") {

            } else {

            }

            //now go run the autonomous method to move the robot
            runAutonomous();

        while (!isStopRequested() && opModeIsActive()) ;
    }

    public void runAutonomous() {

        /* do the stuff */


    }
    public void initVision() {
        //necessary for all auto code that uses vision
        vision.init(hardwareMap);

        pipeline = new OpenCVUtil.SkystoneDeterminationPipeline();

        vision.webcam.setPipeline(pipeline);
        vision.webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.

        vision.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                vision.webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

}
