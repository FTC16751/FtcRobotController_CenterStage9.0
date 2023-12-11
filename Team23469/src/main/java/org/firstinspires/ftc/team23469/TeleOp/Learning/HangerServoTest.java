/* P3 2021-22 season Driver Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.team23469.TeleOp.Learning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team23469.robot.utilities.Learning.hangerServoUtil;


@TeleOp(name="Hanger Servo Test", group="Teleop")
@Disabled
public class HangerServoTest extends LinearOpMode {
    hangerServoUtil hangerservo = new hangerServoUtil(this);

    @Override

    //Initialize and run program
    public void runOpMode() throws InterruptedException {
        hangerservo.init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            dohangerservo();
        } //end OpModeIsActive
    }  //end runOpMode

    public void dohangerservo() {
        if (gamepad2.left_bumper){
            hangerservo.setServoUp();
            telemetry.addData("launch up", "call up launcher");
            telemetry.update();

        }
        else {//does nothing
            }

        if (gamepad2.right_bumper){
            hangerservo.setServoDown();
        }
        else {// continue looping
        }

        if(gamepad2.x) {
            hangerservo.setServoPosition(.3);
        } else if (gamepad2.y) {
            hangerservo.setServoPosition(-.3);
        } else if (gamepad2.b) {
            hangerservo.setServoPosition(1);
        } else if (gamepad2.a) {
            hangerservo.setServoPosition(-1);
        } else { }
    }

} //end program
