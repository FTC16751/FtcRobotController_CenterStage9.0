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
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.team16751.robot.utilities.ArmUtil;

@TeleOp(name="Arm Motor Test", group="Testing")
@Disabled
public class ArmMotorTest extends LinearOpMode {

    //MotorLiftTest lift = new MotorLiftTest(this);
    ArmUtil arm = new ArmUtil(this);

    double DRIVE_SPEED = 1;
    double handOffset   = 0;
    DcMotor elbow;
    int elbowStowPosition;
    int elbowAcquire;
    int elbowLowScore;
    int elbowTransport;
    int elbowPosition;

    DcMotor shoulder;
    int shoulderStowPosition;
    int shoulderAcquire;
    int shoulderLowScore;
    int shoulderPosition3;
    int shoulderPosition;
    enum ArmState {
        START,
        ACQUIRE,
        LOWSCORE,
        TRANSPORT,
    }
    ArmState state = ArmState.START;
    @Override

    //Initialize and run program
    public void runOpMode()  throws InterruptedException {
        //update status on driver station
        telemetry.addData("Status", "Initializing...");

        arm.init(hardwareMap);

        elbow = hardwareMap.get(DcMotor.class, "Elbow");
        elbow.setDirection(DcMotor.Direction.REVERSE);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int minPosition = 0;
        int maxPosition = (int)(100);
        elbowStowPosition = (int)(0);
        elbowAcquire = (int)(50);
        elbowLowScore = (int)(100);
        elbowTransport = (int)(0);

        shoulder = hardwareMap.get(DcMotor.class, "Shoulder");
        shoulder.setDirection(DcMotor.Direction.FORWARD);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int ShoulderminPosition = 0;
        int ShouldermaxPosition = (int)(500);
        shoulderStowPosition = (int)(0);
        shoulderAcquire = (int)(104);
        shoulderLowScore = (int)(82);
        shoulderPosition3 = (int)(0);

        telemetry.addData("Arm Position reset: ",  "Starting at %7d", arm.getMotorPosition());
        telemetry.addData("Status", "Initialized.  Press Play.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            //telemetry
            telemetry.addData("Status: ", "Running.");
            telemetry.update();
            //genericMotor();

            switch (state) {
                case START:
                    telemetry.addData("State", "Start");

                    //  shoulderPosition = shoulderRest;
                //    shoulder.setTargetPosition(shoulderPosition);
                 //   shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    shoulder.setPower(1);

                    //  = elbowRest;
                    elbow.setTargetPosition(elbowStowPosition);
                    elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elbow.setPower(1);
                    if (gamepad2.dpad_left) {
                        state = ArmState.ACQUIRE;
                    }
                    else if (gamepad2.dpad_up) {
                        state = ArmState.TRANSPORT;
                    }
                    else if (gamepad2.dpad_right) {
                        state = ArmState.LOWSCORE;
                    }
                    else if (gamepad2.dpad_down) {
                        state = ArmState.START;
                    }
                    else {}
                    break;
                case ACQUIRE:
                    //shoulderPosition = shoulderAcquire;
                    telemetry.addData("State", "Acquire");

                   // shoulder.setTargetPosition(shoulderPosition);
                  //  shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                  //  shoulder.setPower(1);

                    // elbow = elbow;
                    elbow.setTargetPosition(elbowAcquire);
                    elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elbow.setPower(.50);
                    if (gamepad2.dpad_left) {
                        state = ArmState.ACQUIRE;
                    }
                    else if (gamepad2.dpad_up) {
                        state = ArmState.TRANSPORT;
                    }
                    else if (gamepad2.dpad_right) {
                        state = ArmState.LOWSCORE;
                    }
                    else if (gamepad2.dpad_down) {
                        state = ArmState.START;
                    }
                    else {}
                    break;
                case TRANSPORT:
                    //shoulderPosition = shoulderPosition2;
                    telemetry.addData("State", "Trasport");

  //                  shoulder.setTargetPosition(shoulderStowPosition);
    //                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      //              shoulder.setPower(1);

                    //elbow = elbowPosition2;
                    elbow.setTargetPosition(elbowTransport);
                    elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elbow.setPower(1);
                    if (gamepad2.dpad_left) {
                        state = ArmState.ACQUIRE;
                    }
                    else if (gamepad2.dpad_up) {
                        state = ArmState.TRANSPORT;
                    }
                    else if (gamepad2.dpad_right) {
                        state = ArmState.LOWSCORE;
                    }
                    else if (gamepad2.dpad_down) {
                        state = ArmState.START;
                    }
                    else {}
                    break;

                case LOWSCORE:
                    //shoulderPosition = shoulderPosition3;
                    telemetry.addData("State", "Lowscore");

        //            shoulder.setTargetPosition(shoulderLowScore);
          //          shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //        shoulder.setPower(1);

                    //elbowPosition = elbowPosition3;
                    elbow.setTargetPosition(elbowLowScore);
                    elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elbow.setPower(1);
                    if (gamepad2.dpad_left) {
                        state = ArmState.ACQUIRE;
                    }
                    else if (gamepad2.dpad_up) {
                        state = ArmState.TRANSPORT;
                    }
                    else if (gamepad2.dpad_right) {
                        state = ArmState.LOWSCORE;
                    }
                    else if (gamepad2.dpad_down) {
                        state = ArmState.START;
                    }
                    else {}
                    break;
                default:
                    telemetry.addData("State", "Finished");
            }
            } //end OpModeIsActive
    }  //end runOpMode


} //end program
