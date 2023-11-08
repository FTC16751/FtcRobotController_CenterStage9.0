/* P3 2021-22 season Driver Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.team23469.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team23469.robot.utilities.DriveUtil2023;
import org.firstinspires.ftc.team23469.robot.utilities.launcher_coachgeorge;


@Disabled
@TeleOp(name="Driver Control Demo", group="Teleop")

public class SetLauncherServo_George extends LinearOpMode {
    DriveUtil2023 drive = new DriveUtil2023(this);
    launcher_coachgeorge launcher = new launcher_coachgeorge(this);

    @Override

    //Initialize and run program
    public void runOpMode() throws InterruptedException {
        launcher.init();

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
        }
        else {
            // continue looping//
        }

        if (gamepad2.right_bumper){
            launcher.setLauncherDown();
        }
        else {
            // continue looping
        }
    }
} //end program
