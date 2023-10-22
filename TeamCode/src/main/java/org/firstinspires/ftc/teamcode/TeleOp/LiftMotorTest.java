/* P3 2021-22 season Driver Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.utilities.LiftUtil;

@Disabled
@TeleOp(name="Lift Motor Test", group="Testing")
public class LiftMotorTest extends LinearOpMode {

    //MotorLiftTest lift = new MotorLiftTest(this);
    LiftUtil lift = new LiftUtil(this);

    DcMotor BelaArm;
    int armStowPosition = (int)(COUNTS_PER_DEGREE * 35);
    int armLevel1Position = (int)(COUNTS_PER_DEGREE * 60);;
    int armLevel2Position = (int)(COUNTS_PER_DEGREE * 90);;
    int armLevel3Position = (int)(COUNTS_PER_DEGREE * 90);;
    int armPosition;
    int minPosition = 0;
    int maxPosition = (int)(COUNTS_PER_DEGREE * 270);
    ///DcMotor IntakeMotor;

    static final double COUNTS_PER_MOTOR_REV = 2450;
    static final double GEAR_REDUCTION = 1.0;
    static final double COUNTS_PER_GEAR_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
    static final double COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV/360;



    @Override

    //Initialize and run program
    public void runOpMode() {



        //update status on driver station
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        lift.init(hardwareMap);

        /***
        //Billy Arm
        BelaArm = hardwareMap.get(DcMotor.class, "billyarm");
        BelaArm.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized.  Press Play.");
        telemetry.update();

        BelaArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BelaArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/
        telemetry.addData("Arm Position reset: ",  "Starting at %7d", lift.getMotorPosition());
        telemetry.addData("Status", "Initialized.  Press Play.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            //telemetry
            telemetry.addData("Status: ", "Running.");
            telemetry.update();
            doArmLift();
        } //end OpModeIsActive
    }  //end runOpMode


    public void doArmLift() {
        //--------------------------------------------------------------------------
        // code for the arm motor
        // Sets arm to set levels depending on button push (requires encoders)
        //--------------------------------------------------------------------------
        double armPower;
        if(gamepad1.left_trigger>0.2) lift.setMotorPower(-0.5);
        else if(gamepad1.right_trigger>0.2) lift.setMotorPower(0.5);
        else lift.setMotorPower(0);

        if (gamepad1.x) lift.changePosition(50);
        if (gamepad1.b) lift.changePosition(80);
        //else lift.changePosition(0);

        if(gamepad1.dpad_left){
            lift.changePosition(1493);
        }
        if(gamepad1.dpad_up){
            lift.changePosition(2838);

        }
        if(gamepad1.dpad_right){
            lift.changePosition(3970);

        }
        if (gamepad1.dpad_down){
            lift.changePosition(0);
        }


        telemetry.addData("Arm Test current position", lift.getMotorPosition());
        telemetry.update();
        //-end of arm motor code----------------------------------------------------
    }

} //end program
