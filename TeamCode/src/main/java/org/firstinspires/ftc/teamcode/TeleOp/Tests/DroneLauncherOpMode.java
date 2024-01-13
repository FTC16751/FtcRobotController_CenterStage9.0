package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot_utilities.LauncherUtil;
@TeleOp(name = "Coach DroneLauncherOpMode", group = "coach")
public class DroneLauncherOpMode extends LinearOpMode {

    private LauncherUtil launcherUtil;

    @Override
    public void runOpMode() {
        launcherUtil = new LauncherUtil(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.right_trigger > 0.5 && !launcherUtil.isLauncherMoving()) {
                launcherUtil.raiseLauncher();

                while (launcherUtil.isLauncherMoving()) {

                }
                sleep(250);
                launcherUtil.setLauncherUp();
            }

            // Other operations or controls can be added here

            // Add a small delay to avoid spamming the system with commands
            sleep(50);
        }
    }
}
