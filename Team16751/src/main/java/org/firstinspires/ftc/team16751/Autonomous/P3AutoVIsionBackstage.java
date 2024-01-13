package org.firstinspires.ftc.team16751.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team16751.robot.utilities.ArmUtil;
import org.firstinspires.ftc.team16751.robot.utilities.ClawUtil;
import org.firstinspires.ftc.team16751.robot.utilities.NewDriveUtil2024;
import org.firstinspires.ftc.team16751.robot.utilities.TeamElementSubsystem;

@Autonomous(name="Auto w Vision Backstage", group="Auto")

public class P3AutoVIsionBackstage extends LinearOpMode {


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
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,3,0.5,5000 );
            drive.driveRobotDistanceStrafeLeft(40,.3);
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,22,0.5,5000 );
            while (drive.motorisBusyLF() || drive.motorisBusyRF()) {
            }
            claw.openRightHand();
            sleep(250);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,6,0.5,5000 );
            drive.rotateByXDegrees(true,90,0.25,5000);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,16,0.5,5000 );
            arm.raiseToPositionNoPID(5,.5,false); //low score
            sleep(1000);
            arm.setWristPosition(0.0);
            drive.driveRobotDistanceStrafeLeft(20,.3);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,9,0.15,5000 );
            while (drive.motorisBusyLF() || drive.motorisBusyRF()) {

            }
            claw.setClawOpen();
            sleep(250);
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,9,0.5,5000 );
            arm.setWristPosition(0.45);
            claw.setClawClosed();
            arm.raiseToPositionNoPID(2,.5,false); //transport
            drive.driveRobotDistanceStrafeRight(50,0.5);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,16,0.15,5000 );
            arm.raiseToPositionNoPID(1,.5,false); //initial
            /*
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,(45/2.54),0.5,5000 );
            drive.rotateByXDegrees(false,45,0.25,5000);
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,(10/2.54),0.5,5000 );
            while (drive.motorisBusyLF() || drive.motorisBusyRF()) {

            }
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

             */
            sleep(2000);
        }
        else if(element_zone==2&&curAlliance=="blue"){
            drive.resetEncoders();
            arm.setWristPosition(0.3);
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,34,0.5,5000 );
            sleep(1000);
            arm.setWristPosition(0.45);
            claw.openRightHand();
            sleep(500);
            arm.setWristPosition(0.3);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,6,0.5,5000 );
            drive.driveRobotDistanceStrafeLeft(10,.5);
            while (drive.motorisBusyLF() || drive.motorisBusyRF()) {}
            claw.closeRightHand();
            drive.rotateByXDegrees(true,90,0.3,5000);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,31,0.5,5000 );

            arm.setWristPosition(0.45);
            arm.raiseToPositionNoPID(5,.5,false); //low score
            sleep(1000);
            arm.setWristPosition(0.0);
            drive.driveRobotDistanceStrafeLeft(5,.3);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,6,0.15,5000 );
            while (drive.motorisBusyLF() || drive.motorisBusyRF()) {
            }
            claw.setClawOpen();
            sleep(1000);
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,9,0.5,5000 );
            arm.setWristPosition(0.45);
            claw.setClawClosed();
            arm.raiseToPositionNoPID(2,.5,false); //transport
            drive.driveRobotDistanceStrafeRight(55,0.5);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,22,0.5,5000 );
            arm.raiseToPositionNoPID(1,.5,false); //initial
            sleep(2000);
        }
        else if (element_zone==3&&curAlliance=="blue") {
            drive.resetEncoders();
            //place pixel
            arm.setWristPosition(0.3);
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,34,0.5,5000 );
            drive.rotateByXDegrees(true,90,0.3,5000);
            sleep(1000);
            arm.setWristPosition(0.45);
            claw.openRightHand();
            sleep(500);
            arm.setWristPosition(0.3);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,6,0.25,5000 );
            claw.closeRightHand();

            //drive to backdrop
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,26,0.5,5000 );

            //score on backdrop
            arm.setWristPosition(0.45);
            arm.raiseToPositionNoPID(5,.5,false); //low score
            sleep(1000);
            arm.setWristPosition(0.0);
            //drive.driveRobotDistanceStrafeLeft(5,.3);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,6,0.15,5000 );
            while (drive.motorisBusyLF() || drive.motorisBusyRF()) {
            }
            claw.setClawOpen();
            sleep(1000);

            //park
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,12,0.5,5000 );
            arm.setWristPosition(0.45);
            claw.setClawClosed();
            arm.raiseToPositionNoPID(2,.5,false); //transport
            drive.driveRobotDistanceStrafeRight(65,0.5);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,22,0.5,5000 );
            arm.raiseToPositionNoPID(1,.5,false); //initial
            sleep(2000);

        }
        else {
        }
        if(element_zone==1&&curAlliance=="red"){
            drive.resetEncoders();
            //place pixel
            arm.setWristPosition(0.3);
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,34,0.5,5000 );
            drive.rotateByXDegrees(false,90,0.3,5000);
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,2,0.5,5000 );

            sleep(1000);
            arm.setWristPosition(0.45);
            claw.openRightHand();
            sleep(500);
            arm.setWristPosition(0.3);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,6,0.25,5000 );
            claw.closeRightHand();

            //drive to backdrop
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,26,0.5,5000 );

            //score on backdrop
            arm.setWristPosition(0.45);
            arm.raiseToPositionNoPID(5,.5,false); //low score
            sleep(1000);
            arm.setWristPosition(0.0);
            drive.driveRobotDistanceStrafeLeft(5,.3);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,6,0.15,5000 );
            while (drive.motorisBusyLF() || drive.motorisBusyRF()) {
            }
            claw.setClawOpen();
            sleep(1000);

            //park
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,12,0.5,5000 );
            arm.setWristPosition(0.45);
            claw.setClawClosed();
            arm.raiseToPositionNoPID(2,.5,false); //transport
            drive.driveRobotDistanceStrafeLeft(50,0.5);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,22,0.5,5000 );
            arm.raiseToPositionNoPID(1,.5,false); //initial
            sleep(2000);
        }
        else if (element_zone==2&&curAlliance=="red"){
            drive.resetEncoders();
            //go place purple pixel
            arm.setWristPosition(0.3);
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,34,0.5,5000 );
            sleep(1000);
            arm.setWristPosition(0.45);
            claw.openRightHand();
            sleep(500);
            arm.setWristPosition(0.3);

            //get out of the way of the pixel
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,6,0.5,5000 );
            drive.driveRobotDistanceStrafeRight(10,.5);
            while (drive.motorisBusyLF() || drive.motorisBusyRF()) {}
            claw.closeRightHand();

            //drive to the backdrop
            drive.rotateByXDegrees(false,90,0.3,5000);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,31,0.5,5000 );

            //score yellow pixel on backdrop
            arm.setWristPosition(0.45);
            arm.raiseToPositionNoPID(5,.5,false); //low score
            sleep(1000);
            arm.setWristPosition(0.0);
            drive.driveRobotDistanceStrafeRight(15,.3);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,6,0.15,5000 );
            while (drive.motorisBusyLF() || drive.motorisBusyRF()) {
            }
            claw.setClawOpen();
            sleep(1000);

            //go park
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,9,0.5,5000 );
            arm.setWristPosition(0.45);
            claw.setClawClosed();
            arm.raiseToPositionNoPID(2,.5,false); //transport
            drive.driveRobotDistanceStrafeLeft(15,0.5);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,22,0.5,5000 );
            arm.raiseToPositionNoPID(1,.5,false); //initial
            sleep(2000);

        }
        else if (element_zone==3&&curAlliance=="red"){
            drive.resetEncoders();
            //place purple pixel
            drive.resetEncoders();
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,3,0.5,5000 );
            drive.driveRobotDistanceStrafeRight(30,.3);
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,22,0.5,5000 );
            while (drive.motorisBusyLF() || drive.motorisBusyRF()) {
            }
            claw.openRightHand();
            sleep(250);

            //get out of way of pixel
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,6,0.5,5000 );

           //drive to backdrop
            drive.rotateByXDegrees(false,90,0.25,5000);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,16,0.5,5000 );

            //score on backdrop
            arm.raiseToPositionNoPID(5,.5,false); //low score
            sleep(1000);
            arm.setWristPosition(0.0);
            drive.driveRobotDistanceStrafeRight(45,.3);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,10,0.15,5000 );
            while (drive.motorisBusyLF() || drive.motorisBusyRF()) {
            }
            claw.setClawOpen();
            sleep(500);

            //park
            drive.movePID(NewDriveUtil2024.Direction.FORWARD,9,0.5,5000 );
            arm.setWristPosition(0.45);
            claw.setClawClosed();
            arm.raiseToPositionNoPID(2,.5,false); //transport
            drive.driveRobotDistanceStrafeLeft(70,0.5);
            drive.movePID(NewDriveUtil2024.Direction.BACKWARD,16,0.15,5000 );
            arm.raiseToPositionNoPID(1,.5,false); //initial
        }
        //}
    }

    private void dotelemetry() {
        telemetry.addData("telemetry:", "start logging");
        telemetry.addData("robot heading: ", drive.getBotHeading());
        telemetry.update();

    }
}

