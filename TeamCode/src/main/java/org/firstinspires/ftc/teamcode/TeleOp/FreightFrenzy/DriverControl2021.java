/* P3 2021-22 season Driver Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.teamcode.TeleOp.FreightFrenzy;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Driver Control 2021", group="Linear Opmode")
@Disabled
public class DriverControl2021 extends LinearOpMode {
    //drive train variables
    DcMotor left_front_motor;
    DcMotor right_front_motor;
    DcMotor left_rear_motor;
    DcMotor right_rear_motor;
    double LEFT_FRONT_POWER;
    double RIGHT_FRONT_POWER;
    double LEFT_REAR_POWER;
    double RIGHT_REAR_POWER;
    double DRIVE_SPEED;
    double L_HYPOTENUSE;
    double R_HYPOTENUSE;
    DcMotor DuckMotor;
    double duckPower;
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



    Servo intake;

    RevBlinkinLedDriver lights;
    int temp = 1;
    //subclass is replacing inherited behavior.
    @Override

    //Initialize and run program
    public void runOpMode() {
        //update status on driver station
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        left_front_motor  = hardwareMap.get(DcMotor.class, "Front_Left");        left_front_motor  = hardwareMap.get(DcMotor.class, "Front_Left");
        right_front_motor = hardwareMap.get(DcMotor.class, "Front_Right");
        left_rear_motor  = hardwareMap.get(DcMotor.class, "Rear_Left");
        right_rear_motor = hardwareMap.get(DcMotor.class, "Rear_Right");

        // Reverse left motors for driving (so it goes counterclockwise to drive forward)
        left_front_motor.setDirection(DcMotor.Direction.REVERSE);
        right_front_motor.setDirection(DcMotor.Direction.FORWARD);
        left_rear_motor.setDirection(DcMotor.Direction.REVERSE);
        right_rear_motor.setDirection(DcMotor.Direction.FORWARD);

        //Set brake condition to full stop (allso can give DcMotor.ZeroPowerBehavior.FLOAT)
        left_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_rear_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_rear_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set Drive Speed
        DRIVE_SPEED = 1;    //1=Full Speed

        DuckMotor = hardwareMap.get(DcMotor.class, "Shoot");
        DuckMotor.setDirection(DcMotor.Direction.REVERSE);
        DuckMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        duckPower = 1;

        //Billy Arm
        BelaArm = hardwareMap.get(DcMotor.class, "billyarm");
        BelaArm.setDirection(DcMotor.Direction.REVERSE);


        intake = hardwareMap.get(Servo.class, "intake");

        telemetry.addData("Status", "Initialized.  Press Play.");
        telemetry.update();

        BelaArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BelaArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Arm Position reset: ",  "Starting at %7d", BelaArm.getCurrentPosition());

        telemetry.addData("Status", "Initialized.  Press Play.");
        telemetry.update();

        int driveMode = 1;

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);

        //intake powered by motor
        //IntakeMotor = hardwareMap.get(DcMotor.class, "intake");
        //IntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        //IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        duckPower = 1;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        /***************************************************************
         * Game Execution Area
         * Runs continous until "STOP" pressed on Driver Station
         ***************************************************************/
        while (opModeIsActive()) {
            //telemetry
            telemetry.addData("Status: ", "Running.");
            telemetry.addData("Drive Speed: ", "%.2f", DRIVE_SPEED);
            telemetry.update();

            //Set driver speed as a percentage of full (normally set to full)
            if (gamepad1.right_bumper & gamepad1.left_bumper) {
                DRIVE_SPEED = .25;
            } else if (gamepad1.left_bumper) {
                DRIVE_SPEED = .5;
            } else if (gamepad1.right_bumper) {
                DRIVE_SPEED = .75;
            } else {
                DRIVE_SPEED = .50;
            }

            //do drive stuff
            //set drive mode
            if (gamepad1.start) driveMode = 1;
            if (gamepad1.back) driveMode = 2;

            if (driveMode == 1) arcadeDrive();
            else if (driveMode == 2) tankDrive();
            else tankDrive();

            //do spinner
           // doSpinner();
            //doArmLift();
            //doIntake();







            if(temp == 1){
                //resetStartTime();
                temp = 2;
            }

            if (time < 85) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            }
            else if (time >= 85 && time <= 90) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
            }
            else if (time > 90 && time < 110) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
            }
            else if (time >= 110 && time <= 120) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
            }
            //from 91seconds to 94 seconds
            //(time > 85 && time <= 120)
            else
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        } //end OpModeIsActive
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
    }  //end runOpMode

    public void tankDrive() {
             /* This enables a tank drive-like arrangement where the left_stick controls the left
            wheels and the right_stick controls the right wheels uses it x/y values and hypotenuse
            to assign magnitude to the stick_y and stick_x values.  Avoids divide by 0 by checking
            hypotenuse
             */
        L_HYPOTENUSE = Math.sqrt(gamepad1.left_stick_y * gamepad1.left_stick_y +
                gamepad1.left_stick_x * gamepad1.left_stick_x);
        if (L_HYPOTENUSE == 0) {
            LEFT_FRONT_POWER = 0;
            LEFT_REAR_POWER = 0;
        } else {
            LEFT_FRONT_POWER = -gamepad1.left_stick_y *
                    Math.abs(gamepad1.left_stick_y / L_HYPOTENUSE);
            LEFT_FRONT_POWER += gamepad1.left_stick_x *
                    Math.abs(gamepad1.left_stick_x / L_HYPOTENUSE);
            LEFT_REAR_POWER = -gamepad1.left_stick_y *
                    Math.abs(gamepad1.left_stick_y / L_HYPOTENUSE);
            LEFT_REAR_POWER -= gamepad1.left_stick_x *
                    Math.abs(gamepad1.left_stick_x / L_HYPOTENUSE);
        }
        R_HYPOTENUSE = Math.sqrt(gamepad1.right_stick_y * gamepad1.right_stick_y +
                gamepad1.right_stick_x * gamepad1.right_stick_x);
        if (R_HYPOTENUSE == 0) {
            RIGHT_FRONT_POWER = 0;
            RIGHT_REAR_POWER = 0;
        } else {
            RIGHT_FRONT_POWER = -gamepad1.right_stick_y *
                    Math.abs(gamepad1.right_stick_y / R_HYPOTENUSE);
            RIGHT_FRONT_POWER += gamepad1.right_stick_x *
                    Math.abs(gamepad1.right_stick_x / R_HYPOTENUSE);
            RIGHT_REAR_POWER = -gamepad1.right_stick_y *
                    Math.abs(gamepad1.right_stick_y / R_HYPOTENUSE);
            RIGHT_REAR_POWER -= gamepad1.right_stick_x *
                    Math.abs(gamepad1.right_stick_x / R_HYPOTENUSE);
        }

        //Ensure Power is between -1 and 1, then factor down by DRIVE_SPEED
        LEFT_FRONT_POWER = Range.clip(LEFT_FRONT_POWER, -1.0, 1.0) * DRIVE_SPEED;
        LEFT_REAR_POWER = Range.clip(LEFT_REAR_POWER, -1.0, 1.0) * DRIVE_SPEED;
        RIGHT_FRONT_POWER = Range.clip(RIGHT_FRONT_POWER, -1.0, 1.0) * DRIVE_SPEED;
        RIGHT_REAR_POWER = Range.clip(RIGHT_REAR_POWER, -1.0, 1.0) * DRIVE_SPEED;

        // Send calculated power to wheels
        left_front_motor.setPower(LEFT_FRONT_POWER);
        right_front_motor.setPower(RIGHT_FRONT_POWER);
        left_rear_motor.setPower(LEFT_REAR_POWER);
        right_rear_motor.setPower(RIGHT_REAR_POWER);

    }

    public void arcadeDrive() {
        double y = -gamepad1.left_stick_y * DRIVE_SPEED; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1 *DRIVE_SPEED; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x * DRIVE_SPEED;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        //drive.setMotorPowers(frontLeftPower,backLeftPower,backRightPower,frontRightPower);
        left_front_motor.setPower(frontLeftPower);
        right_front_motor.setPower(frontRightPower);
        left_rear_motor.setPower(backLeftPower);
        right_rear_motor.setPower(backRightPower);

        telemetry.addData("Drive Speed: ", "%.2f", frontLeftPower);

    }

    public void doSpinner() {
        //--------------------------------------------------------------------------
        // Duck spinner aka AlexTower
        // code for the arm motor
        // Sets the duck motor to spin either clockwise or counter clockwise
        // uses gamepad2 for control
        //--------------------------------------------------------------------------
        //AlexTower
        if(gamepad2.right_bumper) {
            DuckMotor.setPower(1);
        }
        if(gamepad2.left_bumper) {
            DuckMotor.setPower(1);
        }
        else {
            DuckMotor.setPower(0);
        }

        //-End of Duck spinner aka AlexTower----------------------------------------
    }

    public void doArmLift() {
        //--------------------------------------------------------------------------
        // code for the arm motor
        // Sets arm to set levels depending on button push (requires encoders)
        //--------------------------------------------------------------------------
        double armPower;
 /*       if(gamepad2.dpad_left && BelaArm.getCurrentPosition() < maxPosition){
            armPosition = armLevel1Position;
            BelaArm.setTargetPosition(armPosition);
            BelaArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BelaArm.setPower(1);
        }

        if(gamepad2.dpad_up && BelaArm.getCurrentPosition() < maxPosition){
            armPosition = armLevel2Position;
            BelaArm.setTargetPosition(armPosition);
            BelaArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BelaArm.setPower(1);

        }
        if(gamepad2.dpad_right && BelaArm.getCurrentPosition() < maxPosition){
            armPosition = armLevel3Position;
            BelaArm.setTargetPosition(armPosition);
            BelaArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BelaArm.setPower(1);

        }

        if (gamepad2.dpad_down && BelaArm.getCurrentPosition() > minPosition){
            armPosition = armStowPosition;
            BelaArm.setTargetPosition(armPosition);
            BelaArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BelaArm.setPower(0.5);
        }
        //if {
          //  armPosition =(int)(COUNTS_PER_DEGREE * 5);;
           // BelaArm.setTargetPosition(armPosition);
           // BelaArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           // BelaArm.setPower(0.2);
        //}

  */
        if(gamepad2.left_trigger>0.2) decreasePosition(100);
        if(gamepad2.right_trigger>0.2) increasePosition(100);
        telemetry.addData("Arm Test current position", BelaArm.getCurrentPosition());
        telemetry.addData("minposition ", minPosition);
        telemetry.addData("maxposition", maxPosition);
        // telemetry.update();

        if(gamepad2.back) BelaArm.setPower(0.2);;
        if(gamepad2.start) resetEncoder();
        //if(gamepad2.dpad_down)  belaArm.raiseToPosition(1,.7);
        //if(gamepad2.dpad_left) belaArm.raiseToPosition(2,.7);
        //if(gamepad2.dpad_up) belaArm.raiseToPosition(3,.7);
        //if(gamepad2.dpad_right) belaArm.raiseToPosition(4,.7);
        //if(gamepad2.left_trigger>0.2) decreasePosition(100);
        //if(gamepad2.right_trigger>0.2) increasePosition(100);

        telemetry.addData("Arm Test current position", BelaArm.getCurrentPosition());
        //-end of arm motor code----------------------------------------------------
    }
    public void resetEncoder(){
        BelaArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void increasePosition(int increaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = BelaArm.getCurrentPosition();
        newPosition = currentPosition + increaseAmount;
        changePosition(newPosition);
    }

    public void decreasePosition(int decreaseAmount) {
        int currentPosition;
        int newPosition;
        currentPosition = BelaArm.getCurrentPosition();
        newPosition = currentPosition - decreaseAmount;
        changePosition(newPosition);
    }
    public void changePosition(int newPosition) {
        BelaArm.setTargetPosition(newPosition);
        BelaArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BelaArm.setPower(0.7);
    }

    public void doIntake() {
        // add code for intake spin
        double IntakePower = 0;
        if (gamepad2.y){
            IntakePower = 1;
        }
        else if (gamepad2.a) {
            IntakePower = -1;
        }
        else IntakePower = .5;
        intake.setPosition(IntakePower);
        telemetry.addData("Intake Power", IntakePower);
        telemetry.addData("Intake Position ", intake.getPosition());
        telemetry.update();
    }
} //end program
