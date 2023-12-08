/* P3 2021-22 season Driver Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.team23469.TeleOp.Learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team23469.robot.utilities.Production.DriveUtil;
import org.firstinspires.ftc.team23469.robot.utilities.Learning.launcherUtil;


@TeleOp(name="Driver Control Demo", group="Teleop")

public class LauncherDemo_Abby extends LinearOpMode {
    DriveUtil drive = new DriveUtil(this);
    launcherUtil launcher = new launcherUtil(this);

    @Override

    //Initialize and run program
    public void runOpMode() throws InterruptedException {
        launcher.init();


        //init external hardware classes
        drive.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /***************************************************************
         * Game Execution Area
         * Runs continous until "STOP" pressed on Driver Station
         ***************************************************************/
        while (opModeIsActive()) {
            doLauncher();
        } //end OpModeIsActive
    }  //end runOpMode

    public void doLauncher() {
        // Use gamepad left & right Bumpers to open and close the claw
        // Use the SERVO constants defined in RobotHardware class.
        // Each time around the loop, the servos will move by a small amount.
        // Limit the total offset to half of the full travel range
        if (gamepad2.left_bumper){
            launcher.setLauncherUp();
telemetry.addData("launch up", "call up launcher");
telemetry.update();

        }
        else {// continue looping//
        }

        if (gamepad2.right_bumper){
            launcher.setLauncherDown();
        }
        else {// continue looping
        }

        if(gamepad2.x) {
            launcher.setLauncherPositions(.3);
        } else if (gamepad2.y) {
            launcher.setLauncherPositions(-.3);
        } else if (gamepad2.b) {
            launcher.setLauncherPositions(.6);
        } else if (gamepad2.a) {
            launcher.setLauncherPositions(-.6);
        }
    }

} //end program
