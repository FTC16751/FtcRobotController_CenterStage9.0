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

package org.firstinspires.ftc.team23469.robot.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.team23469.TeleOp.DriverControl_CenterStageTEST;

public class ClawUtil {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private Servo claw = null;
    private Servo   rightHand = null;
    private Servo   wrist = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO       =  0.0 ;
    public static final double WRIST_MID_SERVO       =  0.1 ;
    public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    public static       double offset = 0.0;
    enum ClawState {
        OPEN,
        CLOSE
    }

    public enum WristState {
        GRAB,
        DOWN,
        CARRY,
        SCORE
    }
    ClawState clawstate = ClawState.OPEN;
    WristState wriststate = WristState.DOWN;

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
    public void init()    {

        // Define and initialize ALL installed servos.
        claw = myOpMode.hardwareMap.get(Servo.class, "claw");
        wrist = myOpMode.hardwareMap.get(Servo.class, "wrist");
        claw.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);
        //leftHand.setPosition(MID_SERVO);
        //rightHand.setPosition(MID_SERVO);
        //wrist.setPosition(MID_SERVO);
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }
    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */
    public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.3, 0.02);
        claw.setPosition(MID_SERVO + offset);
    }
    public void setClawOpen() {
        claw.setPosition(0.45);
    }

    public void setClawClosed(){
        claw.setPosition(0.25);
    }
    public void setWristGrab() {
        wrist.setPosition(0.1);
    }
    public void setWristDown() {
        wrist.setPosition(0.0);
    }
    public void setWristCarry() {
        wrist.setPosition(0.3);
    }
    public void setWristScore() {
        wrist.setPosition(0.2);
    }
    public boolean clawisDoneMoving(double position) {
        if (claw.getPosition() == position) {
            return true;
        } else return false;

    }

    public void toggleWrist(WristState wriststate) {
        switch (wriststate) {
            case DOWN:
                setWristDown();
                break;
            case GRAB:
                setWristGrab();
                break;
            case CARRY:
                setWristCarry();
                break;
            case SCORE:
                setWristScore();
                break;
            default:
                break;
        }
    }
}
