package org.firstinspires.ftc.team23469.Autonomous.Learning;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.team23469.robot.utilities.Production.DriveUtil;
import org.firstinspires.ftc.team23469.robot.utilities.Learning.TeamElementSubsystem;

@Autonomous(name="OpenCVAuto", group="Auto")
@Disabled
public class OpenCVAuto extends LinearOpMode {

    public int element_zone = 1;

    private TeamElementSubsystem teamElementDetection = null;

    boolean togglePreview = true;
    private DriveUtil driveUtil = null;

//Initializing Hardware
    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        teamElementDetection = new TeamElementSubsystem(hardwareMap);
        driveUtil = new DriveUtil(this);
        driveUtil.init(hardwareMap);
        telemetry.addData("Object Creation", "Done");
        telemetry.update();

    }

//Output on driver control hub with element zone, camera stream, and alliance color
    public void runOpMode() {

        HardwareStart();
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


            if (gamepad1.x) {
                curAlliance = "blue";
            } else if (gamepad1.b) {
                curAlliance = "red";
            }
            teamElementDetection.setAlliance(curAlliance);
            telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
            telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());


            telemetry.update();


            telemetry.addData("Object", "Passed waitForStart");

            telemetry.update();
            //armUtil.raiseToPosition(2,0.5);

        }
      //  while(opModeIsActive()&& !isStopRequested()){
        waitForStart();
            if(element_zone==1){
                driveUtil.driveRobotDistanceForward(73,0.5);
                driveUtil.rotateLeft90Degrees();
                driveUtil.driveRobotDistanceForward(25,0.5);
                driveUtil.driveRobotDistanceBackward(25,0.5);
                driveUtil.rotateRight90Degrees();
                driveUtil.driveRobotDistanceForward(60,0.5);
                driveUtil.rotateLeft90Degrees();
                driveUtil.driveRobotDistanceForward(260,0.5);
                //armUtil.raiseToPosition(3,0.5);
                //armUtil.raiseToPosition(0,0.5);


            }
            else if(element_zone==2){
                driveUtil.driveRobotDistanceForward(80,0.5);
                driveUtil.driveRobotDistanceBackward(15,0.5);
                driveUtil.rotateLeft90Degrees();
                driveUtil.driveRobotDistanceForward(194.38,0.5);
               //armUtil.raiseToPosition(3,0.5);
                //armUtil.raiseToPosition(0,0.5);
                driveUtil.driveRobotDistanceStrafeRight(75,0.5);
                driveUtil.driveRobotDistanceForward(20,0.5);
            }
            else if (element_zone==3){
                driveUtil.driveRobotDistanceForward(73,0.5);
                driveUtil.rotateRight90Degrees();
                driveUtil.driveRobotDistanceBackward(220,0.5);
                driveUtil.driveRobotDistanceStrafeRight(75,0.5);
                driveUtil.driveRobotDistanceBackward(50,0.5);
               //armUtil.raiseToPosition(3,0.5);
                //armUtil.raiseToPosition(0,0.5);
            }
            else {
        }
        //}
        }
    }

