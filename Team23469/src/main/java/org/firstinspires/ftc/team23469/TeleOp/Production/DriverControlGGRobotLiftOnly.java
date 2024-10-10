package org.firstinspires.ftc.team23469.TeleOp.Production;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.team23469.robot.utilities.Production.DriveUtil2024;
import org.firstinspires.ftc.team23469.robot.utilities.Production.LinearSlidesUtil;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Dual Lift Control", group = "TELEOP")
public class DriverControlGGRobotLiftOnly extends LinearOpMode {

        Deadline gamePadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);
        private LinearSlidesUtil linearSlides;
     @Override
        public void runOpMode() throws InterruptedException {
            //update status on driver station
            telemetry.addData("Status", "Initializing...");
            telemetry.update();

            initializeHardware();

            telemetry.addData("Status", "Initializing Complete, waiting for start");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                doSlideControls();
                addTelemetry();
            }
        }
    private void initializeHardware() throws InterruptedException {
       linearSlides = new LinearSlidesUtil(hardwareMap); //initialize the linear slides subsystem
    }

  private void doSlideControls() {

        if(gamepad2.start) {
            linearSlides.resetEncoder();
        }

        if (linearSlides.isTouchSensorPressed()){
            linearSlides.stopSlides();
            linearSlides.resetEncoder();
        }

        if(gamepad2.right_stick_y==1.0) {
            if(!linearSlides.isTouchSensorPressed()) {
                linearSlides.decreasePosition(100);
            } else if (linearSlides.isTouchSensorPressed()) {
                linearSlides.resetEncoder();
            }
        }

        if(gamepad2.right_stick_y==-1.0) linearSlides.increasePosition(50);

        if (gamepad2.dpad_down) {
            linearSlides.setCurrentState(LinearSlidesUtil.SlideState.LEVEL_ZERO);
        } else if (gamepad2.dpad_left) {
            linearSlides.setCurrentState(LinearSlidesUtil.SlideState.LOW_POSITION);
        } else if (gamepad2.dpad_up) {
            linearSlides.setCurrentState(LinearSlidesUtil.SlideState.MID_POSITION);
        } else if (gamepad2.dpad_right) {
            linearSlides.setCurrentState(LinearSlidesUtil.SlideState.HIGH_POSITION);
        }
        linearSlides.runStateMachine();
    }


    private void addTelemetry() {
        // Telemetry to display slide and servo status

        telemetry.update();
    }
}
