package org.firstinspires.ftc.team24030.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team24030.robot.utilities.DriveUtil2023;

@Autonomous(name="Deliver Purple Pixel George", group="Autonomous")
@Disabled
public class deliverPurplePixel_George extends LinearOpMode
{
    DriveUtil2023 drive = new DriveUtil2023(this);

    @Override
    public void runOpMode() throws InterruptedException
    {
        /* initialize the robot hardware */
        drive.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /* Actually do something useful */
        while (opModeIsActive() ) {
           drive.driveRobotDistanceForward(91,.8);
        }
    }//program end
}
