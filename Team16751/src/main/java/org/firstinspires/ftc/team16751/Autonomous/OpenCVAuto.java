package org.firstinspires.ftc.team16751.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team16751.robot.utilities.TeamElementSubsystem;
import org.firstinspires.ftc.team16751.robot.utilities.DriveUtil2023;

@Autonomous(name="OpenCVAuto", group="Auto")

public class OpenCVAuto extends LinearOpMode {

    public int element_zone = 1;

    private TeamElementSubsystem teamElementDetection = null;

    boolean togglePreview = true;
    private DriveUtil2023 driveUtil2023 = null;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        teamElementDetection = new TeamElementSubsystem(hardwareMap);
        driveUtil2023 = new DriveUtil2023(this);
        driveUtil2023.init(hardwareMap);
        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }


    public void runOpMode() {

        HardwareStart();

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

        }
      //  while(opModeIsActive()&& !isStopRequested()){
        waitForStart();
            if(element_zone==1){
                driveUtil2023.driveRobotDistanceForward(73,0.5);
                driveUtil2023.rotateLeft90Degrees();
                driveUtil2023.driveRobotDistanceForward(6,0.5);
                driveUtil2023.driveRobotBackward(0.5);
                driveUtil2023.rotateRight90Degrees();
                driveUtil2023.driveRobotDistanceForward(40,0.5);
                driveUtil2023.rotateLeft90Degrees();
                driveUtil2023.driveRobotDistanceForward(75,0.5);

            }
            else if(element_zone==2){
                driveUtil2023.driveRobotDistanceForward(80,0.5);
                driveUtil2023.driveRobotDistanceBackward(15,0.5);
                driveUtil2023.rotateLeft90Degrees();
                driveUtil2023.driveRobotDistanceForward(194.38,0.5);
                driveUtil2023.driveRobotDistanceStrafeRight(75,0.5);
                driveUtil2023.driveRobotDistanceForward(20,0.5);
            }
            else if (element_zone==3){
                driveUtil2023.driveRobotDistanceForward(73,0.5);
                driveUtil2023.rotateRight90Degrees();
                driveUtil2023.driveRobotDistanceForward(6,0.5);
                driveUtil2023.driveRobotDistanceBackward(100,0.5);
            }
            else {
        }
        //}
        }
    }

