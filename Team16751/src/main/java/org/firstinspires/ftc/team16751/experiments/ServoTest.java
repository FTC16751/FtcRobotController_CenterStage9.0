package org.firstinspires.ftc.team16751.experiments;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a pushbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Servo Test", group = "Concept")
public class ServoTest extends LinearOpMode {

    static final double INCREMENT   = 0.05;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo leftHandservo, righHandServo, wristservo;
    // double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    double left_position = (0.0);
    double  right_position = (0.0);
    double  wrist_position = (0.0);
    boolean rampUp = true;


    @Override
    public void runOpMode() {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        leftHandservo = hardwareMap.get(Servo.class, "Left Hand");
        righHandServo = hardwareMap.get(Servo.class, "Right Hand");
        wristservo = hardwareMap.get(Servo.class, "Wrist");
        //servo.setDirection(Servo.Direction.REVERSE);
        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){

            // slew the servo, according to the rampUp (direction) variable.
            if (gamepad1.y){
                left_position += INCREMENT;
            }
            if (gamepad1.a){
                left_position -= INCREMENT;
            }
            if (gamepad1.x){
                right_position += INCREMENT;
            }
            if (gamepad1.b){
                right_position -= INCREMENT;
            }
            if (gamepad1.dpad_up){
                wrist_position += INCREMENT;
            }
            if (gamepad1.dpad_down){
                wrist_position -= INCREMENT;
            }


            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", left_position);
            telemetry.addData("Servo Position send to servo", "%5.2f", leftHandservo.getPosition());

            telemetry.addData("Right Hand Servo Position", "%5.2f", right_position);
            telemetry.addData("Right Hand Servo Position sent to servo", "%5.2f",righHandServo.getPosition());

            telemetry.addData("Wrist Servo Position", "%5.2f", wrist_position);
            telemetry.addData("Wrist Servo Position sent to servo", "%5.2f", wristservo.getPosition());
            telemetry.update();

            // Set the servo to the new position and pause;
            leftHandservo.setPosition(left_position);
            righHandServo.setPosition(right_position);
            wristservo.setPosition(wrist_position);
            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
