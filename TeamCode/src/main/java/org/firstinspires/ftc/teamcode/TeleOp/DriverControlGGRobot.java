package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.utilities.HangerUtil;
import org.firstinspires.ftc.teamcode.robot.utilities.LinearSlides;
import org.firstinspires.ftc.teamcode.robot.utilities.RobotServos;
import org.firstinspires.ftc.teamcode.robot.utilities.LauncherUtil;
@TeleOp(name = "Coach Driver Control of GG Robot", group = "coach")
public class DriverControlGGRobot extends LinearOpMode {


        private RobotServos robotServos;
        private LinearSlides linearSlides;
        private LauncherUtil launcherUtil;
        private HangerUtil hangerUtil;
        private boolean lastTriggerState = false;

        @Override
        public void runOpMode() {
            robotServos = new RobotServos(hardwareMap);
            linearSlides = new LinearSlides(hardwareMap);
            launcherUtil = new LauncherUtil(hardwareMap);
            hangerUtil = new HangerUtil(hardwareMap);
            robotServos.setWristState(0);
            launcherUtil.setLauncherDown();

            waitForStart();

            while (opModeIsActive()) {
                doLauncher();;
                doClaw();
                doSlides();
                doHanger();
                addTelemetry();
                // Add a small delay to avoid spamming the system with commands
                sleep(50);
            }
        }

    private void doHanger() {
        boolean currentTriggerState = gamepad2.left_trigger > 0.5;

        if (currentTriggerState && !lastTriggerState) {
            hangerUtil.toggleHanger();
        }

        lastTriggerState = currentTriggerState;

        // Manual overrides using left joystick of gamepad2
        double hangerMotorPower = -gamepad2.left_stick_y;
        double hangerServoPosition = gamepad2.left_stick_x;

        if (Math.abs(hangerMotorPower) >0.5 || Math.abs(hangerServoPosition) >0.5) {
            hangerUtil.setManualMove();
            hangerUtil.moveHangerManually(hangerMotorPower);
            hangerUtil.moveHangerServoManually(hangerServoPosition);
        } else {
            hangerUtil.stopMotor();
        }
    }

    private void addTelemetry() {
        // Telemetry to display slide and servo status
        telemetry.addData("Slide State", linearSlides.getCurrentState());
        telemetry.addData("Claw Position", robotServos.getClawPosition());
        telemetry.addData("Wrist Position", robotServos.getWristPosition());

        // Telemetry for debuggin
        telemetry.addLine("Hanger Telemetry");
        telemetry.addData("Hanger State", hangerUtil.getCurrentState());
        telemetry.addData("Hanger Motor Position", hangerUtil.getHangerPosition());
        telemetry.addData("Hanger Servo Position", hangerUtil.getHangerServoPosition());
        telemetry.addData("Requested Hanger Servo Position", gamepad2.left_stick_x);
        telemetry.addData("left lift position", linearSlides.getLeftLiftPosition());
        telemetry.addData("right lift position", linearSlides.getRightLiftPosition());

        telemetry.update();
    }

    private void doSlides() {
        if (gamepad2.dpad_down) {
            linearSlides.setCurrentState(LinearSlides.SlideState.LEVEL_ZERO);
        } else if (gamepad2.dpad_left) {
            linearSlides.setCurrentState(LinearSlides.SlideState.LOW_POSITION);
        } else if (gamepad2.dpad_up) {
            linearSlides.setCurrentState(LinearSlides.SlideState.MID_POSITION);
        } else if (gamepad2.dpad_right) {
            linearSlides.setCurrentState(LinearSlides.SlideState.HIGH_POSITION);
        }

        linearSlides.runStateMachine();
    }

    private void doClaw() {
        robotServos.toggleClawWithBumper(gamepad2.left_bumper);
        robotServos.toggleWristWithBumper(gamepad2.right_bumper);
    }

    public void doLauncher(){
            if (gamepad2.right_trigger > 0.5 && !launcherUtil.isLauncherMoving()) {
                launcherUtil.raiseLauncher();

                while (launcherUtil.isLauncherMoving()) {

                }
                sleep(250);
                launcherUtil.setLauncherUp();
            }
        }
}
