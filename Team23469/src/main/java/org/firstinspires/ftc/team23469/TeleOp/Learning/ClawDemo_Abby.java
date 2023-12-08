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

import org.firstinspires.ftc.team23469.robot.utilities.Learning.ClawUtil;


@TeleOp(name="Claw Demo", group="Teleop")

public class ClawDemo_Abby extends LinearOpMode {
    ClawUtil claw = new ClawUtil(this);

    @Override

    //Initialize and run program
    public void runOpMode() throws InterruptedException {
        claw.init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /***************************************************************
         * Game Execution Area
         * Runs continous until "STOP" pressed on Driver Station
         ***************************************************************/
        while (opModeIsActive()) {
            doClaw();
        } //end OpModeIsActive
    }  //end runOpMode

    public void doClaw() {
        if (gamepad2.x){
            claw.setClawOpen();
        }
        else {// continue looping//
        }

        if (gamepad2.b) {
            claw.setClawClosed();
        }
        else {// continue looping
        }
        if (gamepad2.y){
            claw.setWristGrab();
        }
        else {// continue looping//
        }if (gamepad2.a){
            claw.setWristDown();
        }
        else {// continue looping//
        }
    }

} //end program
