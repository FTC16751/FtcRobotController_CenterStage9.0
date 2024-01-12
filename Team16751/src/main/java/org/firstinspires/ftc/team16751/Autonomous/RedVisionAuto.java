package org.firstinspires.ftc.team16751.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team16751.robot.utilities.ArmUtil;
import org.firstinspires.ftc.team16751.robot.utilities.DriveUtil2023;
import org.firstinspires.ftc.team16751.robot.utilities.TeamElementSubsystem;
import org.firstinspires.ftc.team16751.robot.utilities.ClawUtil;
@Autonomous(name="Red Vision Auto", group="Auto")

public class RedVisionAuto extends LinearOpMode{

        public int element_zone = 1;

        private TeamElementSubsystem teamElementDetection = null;

        boolean togglePreview = true;
        private DriveUtil2023 driveUtil2023 = null;
        ArmUtil armUtil = new ArmUtil(this);
        ClawUtil claw = new ClawUtil(this);
        //Initializing Hardware
        public void HardwareStart() {
            telemetry.addData("Object Creation", "Start");
            telemetry.update();

            teamElementDetection = new TeamElementSubsystem(hardwareMap);
            driveUtil2023 = new DriveUtil2023(this);
            driveUtil2023.init(hardwareMap);
            telemetry.addData("Object Creation", "Done");
            telemetry.update();
            armUtil.init(hardwareMap);
            claw.init(hardwareMap);
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
                armUtil.raiseToPosition(2,0.5);

            }
            //  while(opModeIsActive()&& !isStopRequested()){
            waitForStart();
            if(element_zone==1){
                driveUtil2023.driveRobotDistanceForward(73,0.5);
                driveUtil2023.rotateLeft90Degrees();
                driveUtil2023.driveRobotDistanceBackward(220,0.5);
                driveUtil2023.rotateRight90Degrees();
                driveUtil2023.rotateRight90Degrees();
                armUtil.raiseToPosition(3,0.5);
                claw.setClawOpen();
                armUtil.raiseToPosition(0,0.5);
                driveUtil2023.driveRobotDistanceStrafeLeft(75,0.5);
                driveUtil2023.driveRobotDistanceBackward(30,0.5);

                /*driveUtil2023.driveRobotDistanceBackward(25,0.5);
                driveUtil2023.rotateRight90Degrees();
                driveUtil2023.driveRobotDistanceForward(60,0.5);
                driveUtil2023.rotateRight90Degrees();
                driveUtil2023.driveRobotDistanceForward(260,0.5);
                arm.raiseToPosition(3,0.5);
                arm.raiseToPosition(0,0.5);*/


            }
            else if(element_zone==2){
                driveUtil2023.driveRobotDistanceForward(80,0.5);
                driveUtil2023.driveRobotDistanceBackward(15,0.5);
                driveUtil2023.rotateRight90Degrees();
                driveUtil2023.driveRobotDistanceForward(194.38,0.5);
                armUtil.raiseToPosition(3,0.5);
                claw.setClawOpen();
                armUtil.raiseToPosition(0,0.5);
                driveUtil2023.driveRobotDistanceStrafeLeft(75,0.5);
                driveUtil2023.driveRobotDistanceForward(20,0.5);
            }
            else if (element_zone==3){
                driveUtil2023.driveRobotDistanceForward(73,0.5);
                driveUtil2023.rotateRight90Degrees();
                driveUtil2023.driveRobotDistanceBackward(12,0.5);
                driveUtil2023.driveRobotDistanceStrafeLeft(75,0.5);
                driveUtil2023.driveRobotDistanceForward(220,0.5);
                armUtil.raiseToPosition(3,0.5);
                claw.setClawOpen();
                armUtil.raiseToPosition(0,0.5);
            }
            else {
            }

        }
    }



