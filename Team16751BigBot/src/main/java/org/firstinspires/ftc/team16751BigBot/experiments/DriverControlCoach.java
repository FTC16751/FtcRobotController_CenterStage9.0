/* P3 2021-22 season Driver Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.team16751BigBot.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.team16751BigBot.robot.utilities.DriveUtil2023;


@TeleOp(name="Testing Testing", group="Teleop")
@Disabled
public class DriverControlCoach extends LinearOpMode {
    DriveUtil2023 drive = new DriveUtil2023(this);


    int temp = 1;
    double DRIVE_SPEED = 1;
    double handOffset   = 0;
    DcMotor ElbowArm;
    int elbowStowPosition;
    int elbowPosition1;
    int elbowPosition2;
    int elbowPosition3;
    int elbowPosition;

    DcMotor ShoulderArm;
    int shoulderStowPosition;
    int shoulderPosition1;
    int shoulderPosition2;
    int shoulderPosition3;
    int shoulderPosition;

    enum ArmState {
        START,
        ACQUIRE,
        PICK,
        TRANSPORT,
        SCORE,
        STOP
    }

    ArmState state = ArmState.START;
    @Override


    //Initialize and run program
    public void runOpMode() throws InterruptedException {
        //update status on driver station
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        //init external hardware classes
        drive.init(hardwareMap);


        ElbowArm = hardwareMap.get(DcMotor.class, "Elbow");
        ElbowArm.setDirection(DcMotor.Direction.REVERSE);
        int minPosition = 0;
        int maxPosition = (int)(12000);
        elbowStowPosition = (int)(0);
        elbowPosition1 = (int)(100);
        elbowPosition2 = (int)(0);
        elbowPosition3 = (int)(100);

        ShoulderArm = hardwareMap.get(DcMotor.class, "Shoulder");
        ShoulderArm.setDirection(DcMotor.Direction.FORWARD);
        int ShoulderminPosition = 0;
        int ShouldermaxPosition = (int)(500);
        shoulderStowPosition = (int)(0);
        shoulderPosition1 = (int)(20);
        shoulderPosition2 = (int)(20);
        shoulderPosition3 = (int)(20);

        //default drive move to 1 (arcade)
        int driveMode = 1;


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /***************************************************************
         * Game Execution Area
         * Runs continous until "STOP" pressed on Driver Station
         ***************************************************************/
        while (opModeIsActive()) {

            //Set driver speed as a percentage of full (normally set to full)
            if (gamepad1.right_bumper & gamepad1.left_bumper) {
                DRIVE_SPEED = .25;
            } else if (gamepad1.left_bumper) {
                DRIVE_SPEED = .50;
            } else if (gamepad1.right_bumper) {
                DRIVE_SPEED = 1.00;
            } else {
                DRIVE_SPEED = .80;
            }

            if (gamepad1.x) {
                drive.driveRobotStrafeLeft(1.0);
            }
            if (gamepad1.b) {
                drive.driveRobotStrafeRight(1.0);
            }


            /***************************************************************
             * Set Drive Mode
             * This section of code will allow the driver to set
             * the drive mode between arcade drive and tank drive
             * arcade drive is set when clicking the start button on the controller
             * tank drive is set when clicking the back button on the controller
             * this can be done while in teleop operation
             * default drive mode is arcade drive
             ***************************************************************/
            if (gamepad1.start) driveMode = 1;
            if (gamepad1.back) driveMode = 2;

            //call the respective drive mode
            if (driveMode == 1) {
                arcadeDrive();
                telemetry.addData("Drive Mode", "Arcade");
            }
            else if (driveMode == 2) {
                tankDrive();
                telemetry.addData("Drive Mode", "Tank");
            }
            else {
                arcadeDrive();
                telemetry.addData("Drive Mode", "Arcade");
            }

            /***end of set drive mode code */
            if (gamepad2.left_bumper){

            } else {// continue looping//
            }

            if (gamepad2.right_bumper){

            }else {// continue looping
            }
/*
            //ELBOW ARM CODE
            if(gamepad2.dpad_left && ElbowArm.getCurrentPosition() < maxPosition){
                elbowPosition = elbowPosition1;
                ElbowArm.setTargetPosition(elbowPosition);
                ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElbowArm.setPower(1);
            }
            if(gamepad2.dpad_up && ElbowArm.getCurrentPosition() < maxPosition){
                elbowPosition = elbowPosition2;
                ElbowArm.setTargetPosition(elbowPosition);
                ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElbowArm.setPower(1);

            }
            if(gamepad2.dpad_right && ElbowArm.getCurrentPosition() < maxPosition){
                elbowPosition = elbowPosition3;
                ElbowArm.setTargetPosition(elbowPosition);
                ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElbowArm.setPower(1);

            }
            if (gamepad2.dpad_down && ElbowArm.getCurrentPosition() > minPosition){
                elbowPosition = elbowStowPosition;
                ElbowArm.setTargetPosition(elbowPosition);
                ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElbowArm.setPower(1);
            }


            //Shoulder ARM CODE
            if(gamepad2.x && ShoulderArm.getCurrentPosition() < ShouldermaxPosition){
                shoulderPosition = shoulderPosition1;
                ShoulderArm.setTargetPosition(shoulderPosition);
                ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ShoulderArm.setPower(1);
            }
            if(gamepad2.y && ShoulderArm.getCurrentPosition() < ShouldermaxPosition){
                shoulderPosition = shoulderPosition2;
                ShoulderArm.setTargetPosition(shoulderPosition);
                ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ShoulderArm.setPower(1);

            }
            if(gamepad2.b && ShoulderArm.getCurrentPosition() < ShouldermaxPosition){
                shoulderPosition = shoulderPosition3;
                ShoulderArm.setTargetPosition(shoulderPosition);
                ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ShoulderArm.setPower(1);

            }
            if (gamepad2.a && ShoulderArm.getCurrentPosition() > minPosition){
                shoulderPosition = shoulderStowPosition;
                ShoulderArm.setTargetPosition(shoulderPosition);
                ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ShoulderArm.setPower(1);
            }

 */
            switch (state) {
                case START:

                    shoulderPosition = shoulderStowPosition;
                    ShoulderArm.setTargetPosition(shoulderPosition);
                    ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ShoulderArm.setPower(1);

                    elbowPosition = elbowStowPosition;
                    ElbowArm.setTargetPosition(elbowPosition);
                    ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ElbowArm.setPower(1);
                    if (gamepad2.dpad_left) {
                        state = ArmState.ACQUIRE;
                    }
                    else if (gamepad2.dpad_up) {
                        state = ArmState.TRANSPORT;
                    }
                    else if (gamepad2.dpad_right) {
                        state = ArmState.SCORE;
                    }
                    else if (gamepad2.dpad_down) {
                        state = ArmState.STOP;
                    }
                    else {}
                    break;
                case ACQUIRE:
                    shoulderPosition = shoulderPosition1;
                    ShoulderArm.setTargetPosition(shoulderPosition);
                    ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ShoulderArm.setPower(1);

                    elbowPosition = elbowPosition1;
                   // ElbowArm.setTargetPosition(elbowPosition);
                   // ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                   // ElbowArm.setPower(1);

                    if ((ShoulderArm.getCurrentPosition() >= shoulderPosition-1
                        && ShoulderArm.getCurrentPosition() <= shoulderPosition+1)
                            && (ElbowArm.getCurrentPosition() >= elbowPosition-1
                            && ElbowArm.getCurrentPosition() <= elbowPosition+1)
                    ) {
                        state = ArmState.PICK;
                    }
                    break;
                case PICK:
                    if (gamepad2.dpad_left) {
                        state = ArmState.ACQUIRE;
                    }
                    else if (gamepad2.dpad_up) {
                        state = ArmState.TRANSPORT;
                    }
                    else if (gamepad2.dpad_right) {
                        state = ArmState.SCORE;
                    }
                    else if (gamepad2.dpad_down) {
                        state = ArmState.STOP;
                    }
                    else {}
                    break;
                case TRANSPORT:
                    shoulderPosition = shoulderPosition2;
                    ShoulderArm.setTargetPosition(shoulderPosition);
                    ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ShoulderArm.setPower(1);

                    elbowPosition = elbowPosition2;
                    ElbowArm.setTargetPosition(elbowPosition);
                    ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ElbowArm.setPower(1);

                    if (gamepad2.dpad_left) {
                        state = ArmState.ACQUIRE;
                    }
                    else if (gamepad2.dpad_up) {
                        state = ArmState.TRANSPORT;
                    }
                    else if (gamepad2.dpad_right) {
                        state = ArmState.SCORE;
                    }
                    else if (gamepad2.dpad_down) {
                        state = ArmState.STOP;
                    }
                    else {}

                    break;
                case SCORE:
                    shoulderPosition = shoulderPosition3;
                    ShoulderArm.setTargetPosition(shoulderPosition);
                    ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ShoulderArm.setPower(1);

                    elbowPosition = elbowPosition3;
                    ElbowArm.setTargetPosition(elbowPosition);
                    ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ElbowArm.setPower(1);

                    if (gamepad2.dpad_left) {
                        state = ArmState.ACQUIRE;
                    }
                    else if (gamepad2.dpad_up) {
                        state = ArmState.TRANSPORT;
                    }
                    else if (gamepad2.dpad_right) {
                        state = ArmState.SCORE;
                    }
                    else if (gamepad2.dpad_down) {
                        state = ArmState.STOP;
                    }
                    else {}

                    break;
                case STOP:
                    elbowPosition = elbowStowPosition;
                    ElbowArm.setTargetPosition(elbowPosition);
                    ElbowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ElbowArm.setPower(1);

                    if ((ElbowArm.getCurrentPosition() >= elbowPosition-5
                            && ElbowArm.getCurrentPosition() <= elbowPosition+5)
                    ) {
                        shoulderPosition = shoulderStowPosition;
                        ShoulderArm.setTargetPosition(shoulderPosition);
                        ShoulderArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ShoulderArm.setPower(.25);
                    }


                    if (gamepad2.dpad_left) {
                        state = ArmState.ACQUIRE;
                    }
                    else if (gamepad2.dpad_up) {
                        state = ArmState.TRANSPORT;
                    }
                    else if (gamepad2.dpad_right) {
                        state = ArmState.SCORE;
                    }
                    else if (gamepad2.dpad_down) {
                        state = ArmState.STOP;
                    }
                    else {}

                    break;
                default:
                    telemetry.addData("State", "Finished");
            }
            telemetry.addData("Arm Test current position", ElbowArm.getCurrentPosition());
            telemetry.addData("elbow minposition ", minPosition);
            telemetry.addData("elbow maxposition", maxPosition);
            telemetry.addLine("Shoulder Stats");
            telemetry.addData("Shoulder current position", ShoulderArm.getCurrentPosition());
            telemetry.addData("shoulder minposition ", ShoulderminPosition);
            telemetry.addData("shoulder maxposition", ShouldermaxPosition);
            telemetry.update();

        } //end OpModeIsActive
    }  //end runOpMode

    public void tankDrive() {
        drive.tankDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, DRIVE_SPEED);
    }

    public void arcadeDrive() {
        drive.arcadeDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, DRIVE_SPEED);
    }

} //end program
