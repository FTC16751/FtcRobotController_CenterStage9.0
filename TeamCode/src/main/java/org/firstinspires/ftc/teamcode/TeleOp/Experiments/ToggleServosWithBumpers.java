package org.firstinspires.ftc.teamcode.TeleOp.Experiments;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot_utilities.experiments.RobotServos;

@TeleOp(name = "ToggleServosWithBumpers", group = "coach")
@Disabled
public class ToggleServosWithBumpers extends LinearOpMode {

    private RobotServos robotServos;

    @Override
    public void runOpMode() {
        robotServos = new RobotServos(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            robotServos.toggleClawWithBumper(gamepad2.left_bumper);
            robotServos.toggleWristWithBumper(gamepad2.right_bumper);

            // Other TeleOp controls or logic can be added here
            telemetry.addData("Claw Position", robotServos.getClawPosition());
            telemetry.update();
            idle();
        }
    }
}
