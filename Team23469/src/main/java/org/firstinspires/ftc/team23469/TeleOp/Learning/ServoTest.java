/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.team23469.TeleOp.Learning;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
    Servo   rservo;
    Servo   lservo;
    // double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    double  position = (0.0);
    boolean rampUp = true;

    private Servo rightServoClaw, leftServoClaw;
    private Servo wristServo;
    private Servo launcherServo;
    private Servo launcherAngleServo;
    private Servo rhangerServo;
    private Servo lhangerServo;
    RevBlinkinLedDriver lights, rlights;
    // Default positions
    private final double LAUNCHER_UP_POSITION = 0.2;
    private final double LAUNCHER_DOWN_POSITION = 0.6;
    private final double LAUNCHER_ANGLE_UP_POSITION = 0.1;
    private final double LAUNCHER_ANGLE_DOWN_POSITION = 0.0;
    private double launcherposition = LAUNCHER_ANGLE_DOWN_POSITION;
    private double triggerposition =LAUNCHER_DOWN_POSITION;
    private double lhangerposition = 0.5;
    private double rhangerposition = 0.8;
    private double wristposition = 0.4;
    private double rclawposition = 0.40;
    private double lclawposition = 0.80;

    @Override
    public void runOpMode() {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        rightServoClaw = hardwareMap.servo.get("rclaw");
        leftServoClaw = hardwareMap.servo.get("claw");
        wristServo = hardwareMap.servo.get("wrist");
        wristServo.setDirection(Servo.Direction.REVERSE);
        launcherServo = hardwareMap.servo.get("launcher");
        launcherAngleServo = hardwareMap.servo.get("launcherangle");
        rhangerServo = hardwareMap.servo.get("righthangerservo");
        lhangerServo = hardwareMap.servo.get("lefthangerservo");
        lhangerServo.setPosition(rhangerposition);
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights"); //initialize lights from hub
        rlights = hardwareMap.get(RevBlinkinLedDriver.class, "rlights"); //initialize lights from hub

        //set init states
        leftServoClaw.setPosition(lclawposition);
        rightServoClaw.setPosition(rclawposition);
        wristServo.setPosition(wristposition);
        launcherServo.setPosition(triggerposition);
        launcherAngleServo.setPosition(launcherposition);
        rhangerServo.setPosition(lhangerposition);
        lhangerServo.setPosition(rhangerposition);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN); //once done, set lights to green
        rlights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN); //once done, set lights to green




        //lservo = hardwareMap.get(Servo.class, "lefthangerservo");
        //servo.setDirection(Servo.Direction.REVERSE);
        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){

            // claw
            //left claw
            if (gamepad1.y){
                lclawposition += INCREMENT;
            }
            if (gamepad1.a){
                lclawposition -= INCREMENT;
            }
            leftServoClaw.setPosition(lclawposition);
            telemetry.addData("Left Claw", "%5.2f", leftServoClaw.getPosition());

            //right claw
            if (gamepad1.x){
                rclawposition += INCREMENT;
            }
            if (gamepad1.b){
                rclawposition -= INCREMENT;
            }
            rightServoClaw.setPosition(rclawposition);
            telemetry.addData("Right Claw", "%5.2f", rightServoClaw.getPosition());

            //wrist
            if (gamepad1.right_bumper){
                wristposition += INCREMENT;
            }
            if (gamepad1.left_bumper){
                wristposition -= INCREMENT;
            }
            wristServo.setPosition(wristposition);
            telemetry.addData("Wrist ", "%5.2f", wristServo.getPosition());

            //launcher stuff
            //launcher angle
            if (gamepad1.dpad_up){
                launcherposition += INCREMENT;
            }
            if (gamepad1.dpad_down){
                launcherposition -= INCREMENT;
            }
            launcherAngleServo.setPosition(launcherposition);
            telemetry.addData("Launcher angle ", "%5.2f", launcherAngleServo.getPosition());

            //launcher trigger
            if (gamepad1.dpad_right) {
                triggerposition += INCREMENT;
            }
            if (gamepad1.dpad_left){
                triggerposition -= INCREMENT;
            }
            launcherServo.setPosition(triggerposition);
            telemetry.addData("Launch trigger ", "%5.2f", launcherServo.getPosition());

            //hanger
            if (gamepad1.left_stick_y ==-1.0){
                lhangerposition += INCREMENT;
            }
            if (gamepad1.left_stick_y==1.0){
                lhangerposition -= INCREMENT;
            }
            lhangerServo.setPosition(lhangerposition);
            telemetry.addData("Left hanger ", "%5.2f", lhangerServo.getPosition());

            if (gamepad1.right_stick_y ==-1.0){
                rhangerposition += INCREMENT;
            }
            if (gamepad1.right_stick_y==1.0){
                rhangerposition -= INCREMENT;
            }
            rhangerServo.setPosition(rhangerposition);
            telemetry.addData("Left hanger ", "%5.2f", rhangerServo.getPosition());

           // lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights"); //initialize lights from hub
            //rlights = hardwareMap.get(RevBlinkinLedDriver.class, "rlights"); //initialize lights from hub


            // Display the current value
            telemetry.update();
            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
