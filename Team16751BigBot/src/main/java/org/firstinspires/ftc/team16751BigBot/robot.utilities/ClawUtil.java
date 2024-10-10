/* Copyright (c) 2022 FIRST. All rights reserved.
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

package org.firstinspires.ftc.team16751BigBot.robot.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class ClawUtil {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private Servo   leftHand = null;
    private Servo   rightHand = null;
    private boolean leftClawClosed = false;
    private boolean rightClawClosed = false;
    private boolean lastLeftBumperState = false;
    private boolean lastRightBumperState = false;
    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public ClawUtil(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(HardwareMap hardwareMap)    {

        // Define and initialize ALL installed servos.
        leftHand = myOpMode.hardwareMap.get(Servo.class, "Left Hand");
        rightHand = myOpMode.hardwareMap.get(Servo.class, "Right Hand");

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();

    }

    public void setClawOpen() {
        openLeftHand();
        openRightHand();
//        leftHand.setPosition(0.50);
//        rightHand.setPosition(0.70);
    }
    public void setClawClosed(){
        closeLeftHand();
        closeRightHand();
        //leftHand.setPosition(0.70);
       // rightHand.setPosition(0.40);
    }
   /* public void openLeftHand(){
        leftHand.setPosition(0.30);
    }
    public void closeLeftHand() {
        leftHand.setPosition(0.70);
    }
    public void openRightHand(){
        rightHand.setPosition(0.70);
    }
    public void closeRightHand() {
        rightHand.setPosition(0.30);
    }

    */
    public void openLeftHand(){
        leftHand.setPosition(0.30);
    }
    public void closeLeftHand() {
        leftHand.setPosition(0.70);
    }
    public void openRightHand(){
        rightHand.setPosition(0.70);
    }
    public void closeRightHand() {
        rightHand.setPosition(0.30);
    }
    public void toggleLeftClawWithBumper(boolean leftBumper) {
        boolean leftBumperEdge = leftBumper && !lastLeftBumperState;

        if (leftBumperEdge) {
            // Toggle the claw state when the left bumper is pressed
            if (leftClawClosed) {
                openLeftHand();
            } else {
                closeLeftHand();
            }
        }
        lastLeftBumperState = leftBumper;
    }
    public void toggleRightClawWithBumper(boolean rightBumper) {
        boolean rightBumperEdge = rightBumper && !lastRightBumperState;

        if (rightBumperEdge) {
            // Toggle the claw state when the left bumper is pressed
            if (rightClawClosed) {
                openRightHand();
            } else {
                closeRightHand();
            }
        }
        lastRightBumperState = rightBumper;
    }
    public double getLeftClawPosition(){
       return leftHand.getPosition();

    }
    public double getRightClawPosition(){
        return rightHand.getPosition();
    }
    public double getWristPosition(){
        return getWristPosition();
    }
/*    public void toggleClawWithBumper(boolean leftBumper) {
        boolean leftBumperEdge = leftBumper && !lastLeftBumperState;

        if (leftBumperEdge) {
            // Toggle the claw state when the left bumper is pressed
            if (clawClosed) {
                leftHand.setPosition(0.25);
            } else {
                closeClaw();
            }
        }

        lastLeftBumperState = leftBumper;
    }

 */
}

