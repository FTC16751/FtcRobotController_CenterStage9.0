package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.robot.utilities.LinearSlides;

public class DualLinearSlidesFSM extends LinearOpMode {

    private LinearSlides linearSlides;

    @Override
    public void runOpMode() {
        linearSlides = new LinearSlides(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            linearSlides.runStateMachine();

            // Add a break period between state transitions (for demonstration purposes)
            sleep(2000);
        }
    }
}
