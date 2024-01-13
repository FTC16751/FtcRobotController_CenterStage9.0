package org.firstinspires.ftc.teamcode.robot_utilities.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name="OpenCV Example", group="Example")
public class OpenCVExample extends OpMode {

    OpenCvWebcam webcam1 = null;

    @Override
    public void init(){
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new examplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened(){
                webcam1.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                This will be called if the camera could not be opened
                 */
            }
        });

    }


    @Override
    public void loop(){

    }

    class examplePipeline extends OpenCvPipeline{
        Mat YCbCr = new Mat();
        Mat leftCrop = new Mat();
        Mat rightCrop = new Mat();
        double leftavgfin;
        double rightavfin;
        Mat output = new Mat();
        Scalar rectColor = new Scalar(255.0,0.0,0.0);

        public Mat processFrame(Mat input){

            Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
            Imgproc.cvtColor(input,leftCrop,Imgproc.COLOR_RGB2YCrCb);
            Imgproc.cvtColor(input,rightCrop,Imgproc.COLOR_RGB2YCrCb);

            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1,1,319,359);
            Rect rightRect = new Rect(320, 1,319,359);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect,rectColor,2);
            Imgproc.rectangle(output, rightRect,rectColor,2);

            Core.extractChannel(leftCrop, leftCrop,2);
            Core.extractChannel(rightCrop, rightCrop,2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            rightavfin = rightavg.val[0];
            telemetry.addLine("leftavgfin");
            telemetry.addData("leftavgfin", leftavgfin);
            telemetry.addLine("rightavgfin");
            telemetry.addData("rightavgfin", rightavfin);

            return(output);
        }
    }

}
