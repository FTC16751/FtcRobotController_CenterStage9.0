/* P3 2021-22 season Driver Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.team24030.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team24030.robot.utilities.ArmUtil;

@TeleOp(name="Lift Motor Test", group="Testing")
public class ArmMotorTest extends LinearOpMode {

    //MotorLiftTest lift = new MotorLiftTest(this);
    ArmUtil arm = new ArmUtil(this);

    @Override

    //Initialize and run program
    public void runOpMode() {
        //update status on driver station
        telemetry.addData("Status", "Initializing...");

        arm.init(hardwareMap);

        telemetry.addData("Arm Position reset: ",  "Starting at %7d", arm.getMotorPosition());
        telemetry.addData("Status", "Initialized.  Press Play.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            //telemetry
            telemetry.addData("Status: ", "Running.");
            telemetry.update();
            doArmLift();
            //genericMotor();
        } //end OpModeIsActive
    }  //end runOpMode


    public void doArmLift() {
        //--------------------------------------------------------------------------
        // code for the arm motor
        // Sets arm to set levels depending on button push (requires encoders)
        //--------------------------------------------------------------------------
        if(gamepad1.left_trigger>0.2) arm.setMotorPower(-.5);
        else if(gamepad1.right_trigger>0.2) arm.setMotorPower(.5);
        else arm.lockCurrentPosition();

        if (gamepad1.y) arm.changePosition(0);
        if (gamepad1.a) arm.changePosition(300);
        //else lift.changePosition(0);

        if(gamepad1.dpad_left){
            arm.raiseToPosition(1,1.0);
        }
        if(gamepad1.dpad_up){
            arm.raiseToPosition(2,1.0);
        }
        if(gamepad1.dpad_right){
            arm.raiseToPosition(3,1.0);
        }
        if (gamepad1.dpad_down){
            arm.raiseToPosition(4,1.0);
        }


        telemetry.addData("Arm Test current position", arm.getMotorPosition());
        telemetry.update();
        //-end of arm motor code----------------------------------------------------
    }

} //end program
