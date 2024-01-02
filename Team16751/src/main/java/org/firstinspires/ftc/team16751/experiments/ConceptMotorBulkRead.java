/* Copyright (c) 2019 Phil Malone. All rights reserved.
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

package org.firstinspires.ftc.team16751.experiments;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/*
 * This OpMode illustrates how to use the Expansion Hub's Bulk-Read feature to speed up control cycle times.
 * In this example there are 4 motors that need their encoder positions, and velocities read.
 * The sample is written to work with one or two expansion hubs, with no assumption as to where the motors are located.
 *
 * Three scenarios are tested:
 * Cache Mode = OFF    This is the normal default, where no cache is used, and every read produces a discrete transaction with
 *                     an expansion hub, which is the slowest approach, but guarentees that the value is as fresh (recent) as possible..
 *
 * Cache Mode = AUTO   This mode will attempt to minimize the number of discrete read commands, by performing bulk-reads
 *                     and then returning values that have been cached.  The cache is updated automatically whenever any specific encoder is re-read.
 *                     This mode will always return new data, but it may perform more bulk-reads than absolutely required.
 *                     Extra reads will be performed if multiple encoder/velocity reads are performed on the same encoder in one control cycle.
 *                     This mode is a good compromise between the OFF and MANUAL modes.
 *                     Note: If there are significant user-program delays between encoder reads, the cached value may not be fresh (recent).
 *                     You can issue a clearBulkCache() call at any time force a fresh bulk-read on the next encoder read.
 *
 * Cache Mode = MANUAL This mode requires the user's code to determine the best time to clear the cached bulk-read data.
 *                     Well organized code will reset the cache once at the beginning of the control cycle, and then immediately read and store all the encoder values.
 *                     This approach will produce the shortest cycle times, but it does require the user to manually clear the cache.
 *                     Since NO automatic Bulk-Reads are performed, neglecting to clear the bulk cache will result in the same values being returned
 *                     each time an encoder read is performed.
 *
 * -------------------------------------
 *
 * General tip to speed up your control cycles:
 *
 * No matter what method you use to read encoders and other inputs, you should try to
 * avoid reading the same encoder input multiple times around a control loop.
 * Under normal conditions, this will slow down the control loop.
 * The preferred method is to read all the required inputs ONCE at the beginning of the loop,
 * and save the values in variable that can be used by other parts of the control code.
 *
 * eg: if you are sending encoder positions to your telemetry display, putting a getCurrentPosition()
 * call in the telemetry statement will force the code to go and get another copy which will take time.
 * It's much better read the position into a variable once, and use that variable for control AND display.
 * Reading saved variables takes no time at all.
 *
 * Once you put all your sensor reads at the beginning of the control cycle, it's very easy to use
 * the bulk-read AUTO mode to streamline your cycle timing.
 */
@TeleOp (name = "Motor Bulk Reads", group = "Tests")

public class ConceptMotorBulkRead extends LinearOpMode {
    private DcMotorEx m1, m2, m3, m4; // Motor Objects
    Servo wrist; //Servo Object
    private long      e1, e2, e3, e4; // Encoder Values
    double  position = (0.0);
    static final double INCREMENT   = 0.05;     // amount to slew servo
    @Override
    public void runOpMode() {
        // Important Step 1:  Make sure you use DcMotorEx when you instantiate your motors.
        m1 = hardwareMap.get(DcMotorEx.class, "shoulderLeft");  // Configure the robot to use these 4 motor names,
        m2 = hardwareMap.get(DcMotorEx.class, "shoulderRight");  // or change these strings to match your existing Robot Configuration.
        m3 = hardwareMap.get(DcMotorEx.class, "elbowLeft");
        m4 = hardwareMap.get(DcMotorEx.class, "elbowRight");
        wrist = hardwareMap.get(Servo.class, "Wrist");


        m1.setDirection(DcMotor.Direction.FORWARD);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m2.setDirection(DcMotor.Direction.FORWARD);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m3.setDirection(DcMotor.Direction.FORWARD);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m4.setDirection(DcMotor.Direction.REVERSE);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wrist.setPosition(0.25);

        telemetry.addData(">", "Press play to start tests");
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                m1.getCurrentPosition(),
                m2.getCurrentPosition());
                m3.getCurrentPosition();
                m4.getCurrentPosition();
                wrist.getPosition();
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            // slew the servo, according to the rampUp (direction) variable.
            if (gamepad1.y){
                position += INCREMENT;
            }
            if (gamepad1.a){
                position -= INCREMENT;
            }
            wrist.setPosition(position);
            telemetry.addData("shoulder left location",     m1.getCurrentPosition());
            telemetry.addData("shoulder right location",    m2.getCurrentPosition());
            telemetry.addData("elbow left location",     m3.getCurrentPosition());
            telemetry.addData("elbow right location",    m4.getCurrentPosition());
            telemetry.addData("Wrist Location: ",           wrist.getPosition());
            telemetry.update();
        }


        // wait until op-mode is stopped by user, before clearing display.
        while (opModeIsActive()) ;
    }
}

