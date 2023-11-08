/* P3 2021-22 season Driver Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.team16751.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team16751.robot.utilities.ArmUtil;
import org.firstinspires.ftc.team16751.robot.utilities.GenericMotorUtil;

@TeleOp(name="Lift Motor Test", group="Testing")
public class ArmMotorTest extends LinearOpMode {

    //MotorLiftTest lift = new MotorLiftTest(this);
    ArmUtil arm = new ArmUtil(this);
    GenericMotorUtil motor = new GenericMotorUtil(this);

    @Override

    //Initialize and run program
    public void runOpMode() {
        //update status on driver station
        telemetry.addData("Status", "Initializing...");

        arm.init(hardwareMap);
        motor.init(hardwareMap);

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
        if(gamepad1.left_trigger>0.2) ;
        else if(gamepad1.right_trigger>0.2) arm.setMotorPower(.5);
        else arm.setMotorPower(0);

        if (gamepad1.dpad_left) arm.changePosition(-2);
        if (gamepad1.dpad_up) arm.changePosition(19);
        //else lift.changePosition(0);

        if(gamepad1.dpad_left){
            arm.raiseToPosition(1,.5);
        }
        if(gamepad1.dpad_up){
            arm.raiseToPosition(2,.5);
        }
        if(gamepad1.dpad_right){
            arm.raiseToPosition(3,.5);
        }
        if (gamepad1.dpad_down){
            arm.raiseToPosition(4,.5);
        }


        telemetry.addData("Arm Test current position", arm.getMotorPosition());
        telemetry.update();
        //-end of arm motor code----------------------------------------------------
    }

    public void genericMotor() {
        if (gamepad1.x) {
            motor.setMotorPowers(.4);
        }
    }
} //end program
