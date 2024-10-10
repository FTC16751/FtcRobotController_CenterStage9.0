package org.firstinspires.ftc.team16751BigBot.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team16751BigBot.robot.utilities.ArmUtil;
import org.firstinspires.ftc.team16751BigBot.robot.utilities.ClawUtil;
import org.firstinspires.ftc.team16751BigBot.robot.utilities.DriveUtil2023;
import org.firstinspires.ftc.team16751BigBot.robot.utilities.TeamElementSubsystem;

@Autonomous(name="Camera 1 Vision Test", group="Auto")
@Disabled
public class VisionTestCamera1 extends LinearOpMode {

    public int element_zone = 1;

    private TeamElementSubsystem teamElementDetection = null;

    boolean togglePreview = true;

//Initializing Hardware
    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        teamElementDetection = new TeamElementSubsystem(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();

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


        }

        waitForStart();


        }
    }

