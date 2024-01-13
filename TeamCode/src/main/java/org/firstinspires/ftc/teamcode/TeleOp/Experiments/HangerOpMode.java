package org.firstinspires.ftc.teamcode.TeleOp.Experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot_utilities.HangerUtil;
@TeleOp(name = "Coach Hanger Test on GG Robot", group = "coach")
public class HangerOpMode extends LinearOpMode {

    private HangerUtil hangerUtil;
    private Gamepad gamepad;

    private boolean lastTriggerState = false;

    @Override
    public void runOpMode() {
        hangerUtil = new HangerUtil(hardwareMap);
        gamepad = gamepad2; // Assuming the hanger control is on gamepad2

        waitForStart();

        while (opModeIsActive()) {
            boolean currentTriggerState = gamepad.left_trigger > 0.5;

            if (currentTriggerState && !lastTriggerState) {
                hangerUtil.toggleHanger();
            }

            lastTriggerState = currentTriggerState;

            // Manual overrides using left joystick of gamepad2
            double hangerMotorPower = -gamepad.left_stick_y;
            double hangerServoPosition = gamepad.left_stick_x;

            if (Math.abs(hangerMotorPower) >0.5 || Math.abs(hangerServoPosition) >0.5) {
                hangerUtil.setManualMove();
                hangerUtil.moveHangerManually(hangerMotorPower);
                hangerUtil.moveHangerServoManually(hangerServoPosition);
            } else {
                hangerUtil.stopMotor();
            }

            // Telemetry for debugging
            telemetry.addData("Hanger State", hangerUtil.getCurrentState());
            telemetry.addData("Hanger Motor Position", hangerUtil.getHangerPosition());
            telemetry.addData("Hanger Servo Position", hangerUtil.getHangerServoPosition());
            telemetry.addData("Requested Hanger Servo Position", gamepad.left_stick_x);
            telemetry.update();

            // Add a small delay to avoid spamming the system with commands
            sleep(50);
        }
    }
}
