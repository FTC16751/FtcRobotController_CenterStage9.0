package org.firstinspires.ftc.team16751.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team16751.robot.utilities.ArmUtil;
import org.firstinspires.ftc.team16751.robot.utilities.NewDriveUtil2024;
import org.firstinspires.ftc.team16751.robot.utilities.TeamElementSubsystem;
import org.firstinspires.ftc.team16751.robot.utilities.ClawUtil;

@Autonomous(name="Auto w Vision WING SIDE", group="Auto")

public class P3AutoVisionWing extends LinearOpMode {


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
            drive.rotateByXDegrees(false,90,0.25,5000);
            claw.openRightHand();
            sleep(250);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,(10/2.54),0.5,5000 );
            arm.raiseToPositionNoPID(2,.5, false); //transport
            drive.rotateByXDegrees(false,90,0.25,5000);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,(60/2.54)+6,0.5,5000 );
            drive.rotateByXDegrees(false,90,0.25,5000);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,95,.5,10000 );
            drive.resetEncoders();
            drive.driveRobotDistanceStrafeRight(70,0.5);
            arm.raiseToPositionNoPID(5,.5,false); //low score
            sleep(1000);
            arm.setWristPosition(0.0);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,16,.25,5000);
            sleep(2000);
            claw.setClawOpen();
            sleep(250);
            claw.setClawClosed();
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,6,0.5,5000 );
            arm.raiseToPositionNoPID(2,.5,false); //transport
            arm.setWristPosition(0.45);
            drive.driveRobotDistanceStrafeLeft(80,0.5);
            drive.driveRobotDistanceBackward(50,0.5);
            arm.raiseToPositionNoPID(1,.5,false); //initial
            sleep(2000);
        }
        else if(element_zone==2&&curAlliance=="blue"){
            drive.driveRobotDistanceForward(70,0.5);
            arm.raiseToPositionNoPID(1,0.5, false);
            arm.setWristPosition(0.3);
            claw.openRightHand();
            sleep(1000);
            drive.driveRobotDistanceBackward(5,0.5);
            claw.closeRightHand();
            arm.raiseToPositionNoPID(2,0.5,false);
            arm.setWristPosition(0.3);
            drive.rotateLeft90Degrees();
            drive.driveRobotDistanceForward(205,0.5);
            arm.raiseToPositionNoPID(3,0.5,false);
            arm.setWristPosition(0.3);
            sleep(1000);
            claw.setClawOpen();
            drive.driveRobotDistanceBackward(5,0.5);
            claw.setClawClosed();
            sleep(1000);
            arm.raiseToPositionNoPID(1,.5,false);
            arm.setWristPosition(0.3);
            drive.driveRobotDistanceStrafeRight(60,0.5);
            drive.driveRobotDistanceForward(50,0.5);
        }
        else if (element_zone==3&&curAlliance=="blue") {
            drive.driveRobotDistanceForward(70,0.5);
            drive.rotateRight90Degrees();
            arm.raiseToPositionNoPID(1,0.5,false);
            arm.setWristPosition(0.3);
            claw.openRightHand();
            sleep(200);
            claw.closeRightHand();
            arm.raiseToPositionNoPID(2,0.5,false);
            arm.setWristPosition(0.3);
            drive.driveRobotDistanceBackward(210,0.5);
            drive.driveRobotDistanceStrafeRight(20,0.5);
            arm.raiseToPositionNoPID(5,0.5, false);
            sleep(1000);
            arm.setWristPosition(0.75);
            claw.setClawOpen();
            arm.raiseToPositionNoPID(1,0.5,false);
            sleep(500);
            arm.setWristPosition(0.3);
            drive.driveRobotDistanceStrafeLeft(50,0.5);
            drive.driveRobotDistanceBackward(50,0.5);

        }
        else {
        }
        if(element_zone==1&&curAlliance=="red"){
            drive.driveRobotDistanceForward(70,0.5);
            drive.rotateLeft90Degrees();
            arm.raiseToPositionNoPID(1,0.5,false);
            arm.setWristPosition(0.3);
            claw.openRightHand();
            sleep(1000);
            drive.driveRobotDistanceBackward(10,.5);
            sleep(500);
            claw.closeRightHand();
            arm.raiseToPositionNoPID(2,0.5,false);
            arm.setWristPosition(0.3);
            drive.driveRobotDistanceBackward(205,0.5);
            drive.driveRobotDistanceStrafeRight(20,0.5);
            arm.raiseToPositionNoPID(5,0.5, false);
            sleep(1000);
            arm.setWristPosition(0.45);
            drive.driveRobotDistanceBackward(5,0.5);
            arm.setWristPosition(0.3);
            claw.setClawOpen();
            arm.raiseToPositionNoPID(1,0.5,false);
            sleep(500);
            arm.setWristPosition(0.3);
            drive.driveRobotDistanceStrafeRight(50,0.5);
            drive.driveRobotDistanceBackward(50,0.5);
        }
        else if (element_zone==2&&curAlliance=="red"){
            drive.driveRobotDistanceForward(70,0.5);
            arm.raiseToPositionNoPID(1,0.5, false);
            arm.setWristPosition(0.3);
            claw.openRightHand();
            sleep(1000);
            drive.driveRobotDistanceBackward(5,0.5);
            claw.closeRightHand();
            arm.raiseToPositionNoPID(2,0.5,false);
            arm.setWristPosition(0.3);
            drive.rotateRight90Degrees();
            drive.driveRobotDistanceForward(205,0.5);
            arm.raiseToPositionNoPID(3,0.5,false);
            arm.setWristPosition(0.3);
            sleep(1000);
            claw.setClawOpen();
            sleep(1000);
            claw.setClawClosed();
            sleep(1000);
            arm.raiseToPositionNoPID(1,.5,false);
            arm.setWristPosition(0.3);
            drive.driveRobotDistanceStrafeLeft(60,0.5);
            drive.driveRobotDistanceForward(50,0.5);
        }
        else if (element_zone==3&&curAlliance=="red"){
            //place purple pixel
            drive.driveRobotDistanceForward(73,0.6);
            drive.rotateRight90Degrees();
            drive.driveRobotDistanceForward(5,0.6);
            while (drive.motorisBusyRF() || drive.motorisBusyLF()) {}
            claw.openRightHand();
            arm.raiseToPositionNoPID(2,0.6,false);
            drive.driveRobotDistanceBackward(10,.6);
            drive.rotateLeft90Degrees();
            drive.driveRobotDistanceForward(60,0.6);
            drive.rotateRight90Degrees();

            //drive to backdrop
            drive.driveRobotDistanceForward(180,0.55);
            drive.driveRobotDistanceStrafeRight(90,0.6);

            //score pixel
            arm.raiseToPositionNoPID(3, 0.5,false);
            arm.setWristPosition(0.3);
            drive.driveRobotDistanceForward(20,.25);
            sleep(1000);
            claw.setClawOpen();
            //park
            drive.driveRobotDistanceBackward(15,.6);
            claw.setClawClosed();
            arm.raiseToPositionNoPID(1,0.5,false);
            arm.setWristPosition(0.45);
            drive.driveRobotDistanceStrafeRight(20,0.6);
            drive.driveRobotDistanceForward(20,0.6);
        }
        //}
    }

    private void dotelemetry() {
        telemetry.addData("telemetry:", "start logging");
        telemetry.addData("robot heading: ", drive.getBotHeading());
        telemetry.update();

    }
}

