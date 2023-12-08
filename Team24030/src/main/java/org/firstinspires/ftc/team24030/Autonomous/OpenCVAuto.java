package org.firstinspires.ftc.team24030.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team24030.robot.utilities.ArmUtil;
import org.firstinspires.ftc.team24030.robot.utilities.DriveUtil2023;
import org.firstinspires.ftc.team24030.robot.utilities.TeamElementSubsystem;

@Autonomous(name="OpenCVAuto", group="Auto")

public class OpenCVAuto extends LinearOpMode {

    public int element_zone = 1;

    private TeamElementSubsystem teamElementDetection = null;

    boolean togglePreview = true;
    private DriveUtil2023 driveUtil2023 = null;
    ArmUtil armUtil = new ArmUtil(this);
    private Servo wristServo;
    private Servo leftClaw;
    private Servo rightClaw;
    // Define servo positions for wrist and claws
    private double wristPosition = 0.477; // Initial position
    private double leftClawPosition = 1.0; // Initial position - close
    private double rightClawPosition = 0.0; // Initial position - close

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

        wristServo = hardwareMap.get(Servo.class, "wristservo");
        leftClaw = hardwareMap.get(Servo.class, "leftclaw");
        rightClaw = hardwareMap.get(Servo.class, "rightclaw");

        leftClaw.setDirection(Servo.Direction.REVERSE);
        rightClaw.setDirection(Servo.Direction.REVERSE);
        // Set servo initial positions
        wristServo.setPosition(wristPosition);
        leftClaw.setPosition(leftClawPosition);
        rightClaw.setPosition(rightClawPosition);
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
            telemetry.addData("Object", "Passed waitForStart");
            telemetry.update();
            armUtil.raiseToPosition(2,0.5);

        }
      //  while(opModeIsActive()&& !isStopRequested()){
        waitForStart();
        //wristServo.setPosition(wristPosition);
        // Control claws using bumpers (toggle open/close with Falling Edge Detector)
            //1 is open .x is close
            /*leftClawPosition = (leftClawPosition == 0.6) ? 1.0 : 0.6;
            leftClaw.setPosition(leftClawPosition);

            //0.0 is open .4 is close
            rightClawPosition = (rightClawPosition == 0.0) ? 0.4 : 0.0;
            rightClaw.setPosition(rightClawPosition);
*/

            if(element_zone==1){
                driveUtil2023.driveRobotDistanceForward(73,0.5);
                driveUtil2023.rotateLeft90Degrees();
               /* driveUtil2023.driveRobotDistanceForward(25,0.5);
                driveUtil2023.driveRobotDistanceBackward(25,0.5);
                driveUtil2023.rotateRight90Degrees();
                driveUtil2023.driveRobotDistanceForward(60,0.5);
                driveUtil2023.rotateLeft90Degrees();
                driveUtil2023.driveRobotDistanceForward(260,0.5);

                */
                //armUtil.raiseToPosition(3,0.5);
                //armUtil.raiseToPosition(0,0.5);


            }
            else if(element_zone==2){
                driveUtil2023.driveRobotDistanceForward(80,0.5);
                driveUtil2023.driveRobotDistanceBackward(15,0.5);
                //driveUtil2023.rotateLeft90Degrees();
                /*driveUtil2023.driveRobotDistanceForward(194.38,0.5);
               //armUtil.raiseToPosition(3,0.5);
                //armUtil.raiseToPosition(0,0.5);
                driveUtil2023.driveRobotDistanceStrafeRight(75,0.5);
                driveUtil2023.driveRobotDistanceForward(20,0.5);

                 */
            }
            else if (element_zone==3){
                driveUtil2023.driveRobotDistanceForward(73,0.5);
                driveUtil2023.rotateRight90Degrees();
              /*  driveUtil2023.driveRobotDistanceBackward(220,0.5);
                driveUtil2023.driveRobotDistanceStrafeRight(75,0.5);
                driveUtil2023.driveRobotDistanceBackward(50,0.5);

               */
               //armUtil.raiseToPosition(3,0.5);
              // armUtil.raiseToPosition(0,0.5);
            }
            else {
        }
        //}
        }
    }

