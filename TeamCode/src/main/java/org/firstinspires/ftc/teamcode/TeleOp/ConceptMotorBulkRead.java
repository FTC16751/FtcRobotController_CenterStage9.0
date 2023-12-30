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

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    enum State {
        START,
        RAISE_ELBOW,
        RAISE_SHOULDER,
        STOP
    }
    private ElapsedTime     runtime = new ElapsedTime();
    State state = State.START;
    final int       TEST_CYCLES    = 500;   // Number of control cycles to run to determine cycle times.

    private DcMotorEx m1, m2, m3, m4; // Motor Objects
    private long      e1, e2, e3, e4; // Encoder Values
    private double    v1, v2, v3, v4; // Velocities

    // Cycle Times
    double t1 = 0;
    double t2 = 0;
    double t3 = 0;

    static final double     DRIVE_SPEED             = 0.6;

    @Override
    public void runOpMode() {
        state = State.START;
        int cycles;

        // Important Step 1:  Make sure you use DcMotorEx when you instantiate your motors.
        m1 = hardwareMap.get(DcMotorEx.class, "shoulderLeft");  // Configure the robot to use these 4 motor names,
        m2 = hardwareMap.get(DcMotorEx.class, "shoulderRight");  // or change these strings to match your existing Robot Configuration.
        m3 = hardwareMap.get(DcMotorEx.class, "elbowLeft");  // Configure the robot to use these 4 motor names,
        m4 = hardwareMap.get(DcMotorEx.class, "elbowRight");  // or change these strings to match your existing Robot Configuration.

        // Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        ElapsedTime timer = new ElapsedTime();

        telemetry.addData(">", "Press play to start tests");

        m1.setDirection(DcMotor.Direction.REVERSE);//shoulderLeft
        m2.setDirection(DcMotor.Direction.FORWARD);//shoulderRight
        m3.setDirection(DcMotor.Direction.FORWARD);//elbowLeft
        m4.setDirection(DcMotor.Direction.FORWARD);//elbowRight


        m1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                m1.getCurrentPosition(),
                m2.getCurrentPosition(),
                m3.getCurrentPosition(),
                m4.getCurrentPosition());
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            e1 = m1.getCurrentPosition();
            e2 = m2.getCurrentPosition();
            e3 = m3.getCurrentPosition();
            e4 = m4.getCurrentPosition();
            v1 = m1.getVelocity();
            v2 = m2.getVelocity();

            // Put Control loop action code here.
            telemetry.addData("shoulder left", e1);
            telemetry.addData("shoulder right", e2);
            telemetry.addData("elbow left", e3);
            telemetry.addData("elbow right", e4);
            telemetry.addData("left shoulder direction: ", m1.getDirection());
            telemetry.addData("right shoulder direction: ", m2.getDirection());

            telemetry.addData("left elbow direction: ", m3.getDirection());
            telemetry.addData("right elbow  direction: ", m4.getDirection());

            telemetry.update();

        }


        // wait until op-mode is stopped by user, before clearing display.
        while (opModeIsActive()) ;
    }


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */

   public void encoderDrive(double speed,
                             int M1Target, int M2Target,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = M1Target - m1.getCurrentPosition();
            newRightTarget = M2Target - m2.getCurrentPosition();
            m1.setTargetPosition(newLeftTarget);
            m2.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            m1.setPower(Math.abs(speed));
            m2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (m1.isBusy() && m2.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        m1.getCurrentPosition(), m2.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            m1.setPower(0);
            m2.setPower(0);

            // Turn off RUN_TO_POSITION
            m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    // Display three comparison times.
    void displayCycleTimes(String status) {
        telemetry.addData("Testing", status);
        telemetry.addData("Cache = OFF",    "%5.1f mS/cycle", t1);
        telemetry.addData("Cache = AUTO",   "%5.1f mS/cycle", t2);
        telemetry.addData("Cache = MANUAL", "%5.1f mS/cycle", t3);
        telemetry.update();
    }
}

