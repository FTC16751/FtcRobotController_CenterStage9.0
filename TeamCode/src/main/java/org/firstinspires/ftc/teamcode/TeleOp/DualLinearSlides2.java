package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.utilities.LinearSlidesSimple;

public class DualLinearSlides2 extends LinearOpMode {

    private LinearSlidesSimple linearSlides;

    @Override
    public void runOpMode() {
        linearSlides = new LinearSlidesSimple(hardwareMap);

        waitForStart();

        // Move both slides to different positions in sync
        linearSlides.moveToPosition(linearSlides.getLiftLevelZero());
        sleep(1000);
        linearSlides.moveToPosition(linearSlides.getLiftLowPosition());
        sleep(1000);
        linearSlides.moveToPosition(linearSlides.getLiftMidPosition());
        sleep(1000);
        linearSlides.moveToPosition(linearSlides.getLiftHighPosition());
        sleep(1000);

        // Stop the slides after movement
        linearSlides.stopSlides();
    }
}
