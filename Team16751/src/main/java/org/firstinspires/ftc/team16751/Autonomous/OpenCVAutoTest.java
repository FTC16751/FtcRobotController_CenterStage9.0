package org.firstinspires.ftc.team16751.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team16751.robot.utilities.ArmUtil;
import org.firstinspires.ftc.team16751.robot.utilities.NewDriveUtil2024;
import org.firstinspires.ftc.team16751.robot.utilities.TeamElementSubsystem;
import org.firstinspires.ftc.team16751.robot.utilities.ClawUtil;

@Autonomous(name="OpenCVAuto Test", group="Auto")

public class OpenCVAutoTest extends LinearOpMode {

    public int element_zone = 1;

    private TeamElementSubsystem teamElementDetection = null;

    boolean togglePreview = true;
    private NewDriveUtil2024 drive = null;
    ArmUtil arm = new ArmUtil(this);
    ClawUtil claw = new ClawUtil(this);
    //Initializing Hardware
    public void HardwareStart() {
        teamElementDetection = new TeamElementSubsystem(hardwareMap);
        drive = new NewDriveUtil2024(this);
        drive.init(hardwareMap,telemetry);
        drive.resetIMUYaw();
        arm.init(hardwareMap);
        claw.init(hardwareMap);
        claw.setClawClosed();

    }

    //Output on driver control hub with element zone, camera stream, and alliance color
    public void runOpMode() {

        HardwareStart();
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

            arm.setCurrentState(ArmUtil.ArmState.TRANSPORT);
        }

        waitForStart();
        drive.resetIMUYaw();
        if(element_zone==1&&curAlliance=="blue"){
           //drive.driveRobotDistanceForward(73,0.5);
            drive.resetEncoders();
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,32,0.5,5000 );
            //drive.moveForwardPID(36,.5,5000);

            //drive.driveRobotDistanceForward(73,0.5);
            //drive.rotateLeft90Degrees();
            drive.rotateByXDegrees(false,90,0.5,5000);
            claw.openRightHand();
            sleep(250);
            //drive.driveRobotDistanceBackward(10,.5);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,(10/2.54),0.5,5000 );

            arm.raiseToPosition(2,.5, false); //transport
            //arm.setCurrentState(ArmUtil.ArmState.INIT);
            //drive.rotateRight90Degrees();
            drive.rotateByXDegrees(true,90,0.5,5000);
            //drive.driveRobotDistanceForward(60,0.5);
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,(60/2.54)+6,0.5,5000 );
            //drive.rotateLeft90Degrees();
            drive.rotateByXDegrees(false,90,0.5,5000);
            //drive.turnToHeading(.25,90);

            //drive.driveRobotDistanceForward(200,0.5);
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,80,.5,10000 );
            drive.resetEncoders();
            drive.driveRobotDistanceStrafeLeft(80,0.5);
            drive.turnToHeading(.25,89);
            arm.raiseToPosition(3,.5,false); //low score
            sleep(1000);
            arm.setWristPosition(.3);
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,16,.25,5000);
            sleep(2000);
            claw.setClawOpen();
            sleep(250);
            claw.setClawClosed();
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,6,0.5,5000 );
            arm.raiseToPosition(2,.5,false); //transport
            arm.setWristPosition(0.45);
            drive.driveRobotDistanceStrafeRight(80,0.5);
            drive.driveRobotDistanceForward(50,0.5);
            arm.raiseToPosition(1,.5,false); //initial
            sleep(2000);
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
            arm.setCurrentState(ArmUtil.ArmState.BACK_LOW_SCORE);
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
            arm.setCurrentState(ArmUtil.ArmState.BACK_LOW_SCORE_RAISE_ARM);
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

    private void dotelemetry() {
        telemetry.addData("telemetry:", "start logging");
        telemetry.addData("robot heading: ", drive.getBotHeading());
        telemetry.update();

    }
}

