package org.firstinspires.ftc.team16751BigBot.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16751BigBot.robot.utilities.DriveUtil2023;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name="Simple Autonomous", group="Autonomous")
@Disabled
public class parkBlue extends LinearOpMode
{
    DriveUtil2023 drive = new DriveUtil2023(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException
    {
        /* initialize the robot hardware */
        drive.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        /* ****************************************************
        P3- Your autonomous drive code would start right here
        ******************************************************/
        while (opModeIsActive() ) {
            parkonbluebackstage();
        }
    }//program end

    public void parkonbluebackstage(){
            drive.driveRobotDistanceForward(15,.5);
    }
}
