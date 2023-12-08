package org.firstinspires.ftc.team23469.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team23469.robot.utilities.Production.DriveUtil;

//@TeleOp
@Autonomous(name="Simple Autonomous", group="Autonomous")
@Disabled
public class parkBlue extends LinearOpMode
{
    DriveUtil drive = new DriveUtil(this);
    static final double FEET_PER_METER = 3.28084;
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
        /* Actually do something useful */
        while (opModeIsActive() ) {
            parkonbluebackstage();
        }
    }//program end



    public void parkonbluebackstage(){
            drive.driveRobotDistanceForward(15,.5);
    }
}
