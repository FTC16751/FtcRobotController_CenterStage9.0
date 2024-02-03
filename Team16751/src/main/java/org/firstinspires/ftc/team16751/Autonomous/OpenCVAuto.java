package org.firstinspires.ftc.team16751.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team16751.robot.utilities.ArmUtil;
import org.firstinspires.ftc.team16751.robot.utilities.TeamElementSubsystem;
import org.firstinspires.ftc.team16751.robot.utilities.DriveUtil2023;
import org.firstinspires.ftc.team16751.robot.utilities.ClawUtil;

@Autonomous(name="OpenCVAuto", group="Auto")

public class OpenCVAuto extends LinearOpMode {

    public int element_zone = 1;

    private TeamElementSubsystem teamElementDetection = null;

    boolean togglePreview = true;
    private DriveUtil2023 drive = null;
    ArmUtil arm = new ArmUtil(this);
    ClawUtil claw = new ClawUtil(this);
//Initializing Hardware
    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        teamElementDetection = new TeamElementSubsystem(hardwareMap);
        drive = new DriveUtil2023(this);
        drive.init(hardwareMap);
        telemetry.addData("Object Creation", "Done");
        telemetry.update();
        arm.init(hardwareMap);
        claw.init(hardwareMap);
    }

//Output on driver control hub with element zone, camera stream, and alliance color
    public void runOpMode() {

        HardwareStart();
        telemetry.addData("Object Creation", "Done");
        telemetry.update();
        String curAlliance = "blue";

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
            arm.setCurrentState(ArmUtil.ArmState.TRANSPORT);
        }
      //  while(opModeIsActive()&& !isStopRequested()){
        waitForStart();
            if(element_zone==1&&curAlliance=="blue"){
                drive.driveRobotDistanceForward(73,0.5);
                drive.rotateLeft90Degrees();
                arm.setCurrentState(ArmUtil.ArmState.TRANSPORT);
                claw.openRightHand();
                arm.setCurrentState(ArmUtil.ArmState.INIT);
                drive.rotateRight90Degrees();
                drive.driveRobotDistanceForward(60,0.5);
                drive.rotateLeft90Degrees();
                drive.driveRobotDistanceForward(260,0.5);
                drive.driveRobotDistanceStrafeLeft(100,0.5);
                arm.setCurrentState(ArmUtil.ArmState.LOW_SCORE);
                claw.setClawOpen();
                claw.setClawClosed();
                arm.setCurrentState(ArmUtil.ArmState.INIT);
                drive.driveRobotDistanceStrafeLeft(75,0.5);
                drive.driveRobotDistanceForward(20,0.5);

            }
            else if(element_zone==2&&curAlliance=="blue"){
                /*claw.openRightHand();
                sleep(2000);
                claw.closeRightHand();
                sleep(2000);
                claw.openLeftHand();
                sleep(2000);
                claw.closeLeftHand();
                sleep(26000);*/
               drive.driveRobotDistanceForward(70,0.5);
                //arm.setCurrentState(ArmUtil.ArmState.TRANSPORT);
                claw.openRightHand();
                drive.driveRobotDistanceBackward(5,0.5);
                claw.closeRightHand();
               // arm.raiseToPosition(2,.5);
                //arm.setCurrentState(ArmUtil.ArmState.TRANSPORT);
                //arm.runStateMachine();
                drive.rotateLeft90Degrees();

                drive.driveRobotDistanceForward(180,0.5);
                //arm.setCurrentState(ArmUtil.ArmState.LOW_SCORE);
              //  arm.runStateMachine();
                claw.setClawOpen();
                sleep(1000);
                claw.setClawClosed();
                //arm.setCurrentState(ArmUtil.ArmState.INIT);
              //  arm.runStateMachine();
                drive.driveRobotDistanceStrafeRight(60,0.5);
                drive.driveRobotDistanceForward(60,0.5);
            }
            else if (element_zone==3&&curAlliance=="blue") {
                drive.driveRobotDistanceForward(70,0.5);
                drive.rotateRight90Degrees();
               // arm.setCurrentState(ArmUtil.ArmState.TRANSPORT);
                claw.openRightHand();
                arm.setCurrentState(ArmUtil.ArmState.INIT);
                drive.driveRobotDistanceBackward(190,0.5);
                arm.setCurrentState(ArmUtil.ArmState.BACK_LOW_SCORE_AUTO);
                claw.setClawOpen();
                arm.setCurrentState(ArmUtil.ArmState.INIT);
                drive.driveRobotDistanceStrafeLeft(80,0.5);
                drive.driveRobotDistanceBackward(50,0.5);

            }
            else {
        }
            if(element_zone==1&&curAlliance=="red"){
                drive.driveRobotDistanceForward(70,0.5);
                drive.rotateLeft90Degrees();
                arm.setCurrentState(ArmUtil.ArmState.TRANSPORT);
                arm.runStateMachine();
                claw.openRightHand();
                sleep(200);
                claw.closeRightHand();
                arm.setCurrentState(ArmUtil.ArmState.INIT);
                drive.driveRobotDistanceBackward(190,0.5);
                arm.setCurrentState(ArmUtil.ArmState.BACK_LOW_SCORE_RAISE_ARM_AUTO);
                arm.runStateMachine();
                arm.setCurrentState(ArmUtil.ArmState.BACK_LOW_SCORE_SET_SERVO);
                arm.runStateMachine();
                claw.setClawOpen();
                arm.setCurrentState(ArmUtil.ArmState.INIT);
                arm.runStateMachine();
                drive.driveRobotDistanceStrafeRight(60,0.5);
                drive.driveRobotDistanceBackward(60,0.5);
            }
            else if (element_zone==2&&curAlliance=="red"){
                drive.driveRobotDistanceForward(70,0.5);
               // arm.setCurrentState(ArmUtil.ArmState.TRANSPORT);
                claw.openRightHand();
                sleep(1000);
                drive.driveRobotDistanceBackward(5,0.5);
                claw.closeRightHand();
                //arm.setCurrentState(ArmUtil.ArmState.INIT);
                drive.rotateRight90Degrees();
                drive.driveRobotDistanceForward(180,0.5);
                //arm.setCurrentState(ArmUtil.ArmState.LOW_SCORE);
                claw.setClawOpen();
                sleep(1000);
                claw.setClawClosed();
                //arm.setCurrentState(ArmUtil.ArmState.INIT);
                drive.driveRobotDistanceStrafeLeft(60,0.5);
                drive.driveRobotDistanceForward(60,0.5);
        }

            else if (element_zone==3&&curAlliance=="red"){
                drive.driveRobotDistanceForward(73,0.5);
                drive.rotateLeft90Degrees();
                arm.setCurrentState(ArmUtil.ArmState.TRANSPORT);
                claw.openRightHand();
                arm.setCurrentState(ArmUtil.ArmState.INIT);
                drive.rotateRight90Degrees();
                drive.driveRobotDistanceForward(60,0.5);
                drive.rotateRight90Degrees();
                drive.driveRobotDistanceForward(260,0.5);
                drive.driveRobotDistanceStrafeRight(100,0.5);
                arm.setCurrentState(ArmUtil.ArmState.LOW_SCORE);
                claw.setClawOpen();
                claw.setClawClosed();
                arm.setCurrentState(ArmUtil.ArmState.INIT);
                drive.driveRobotDistanceStrafeRight(75,0.5);
                drive.driveRobotDistanceForward(20,0.5);
            }
        //}
        }
    }

