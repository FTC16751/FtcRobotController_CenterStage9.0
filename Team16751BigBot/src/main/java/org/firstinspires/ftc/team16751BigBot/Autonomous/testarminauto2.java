package org.firstinspires.ftc.team16751BigBot.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16751BigBot.robot.utilities.ArmUtil;
import org.firstinspires.ftc.team16751BigBot.robot.utilities.ClawUtil;

@Autonomous(name="Test Arm in AUto2", group="Auto")
@Disabled
public class testarminauto2 extends LinearOpMode {
    private enum State
    {
        STATE_INITIAL,
        STATE_DRIVE_TO_PIXEL,
        SCORE_PIXEL,
        STATE_STOP,
    }

    private State       currentState;    // Current State Machine State.
    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state
    ArmUtil arm = new ArmUtil(this);
    ClawUtil claw = new ClawUtil(this);
    ArmUtil.ArmState currentArmState;
    //Initializing Hardware


//Output on driver control hub with element zone, camera stream, and alliance color
    public void runOpMode() {
        HardwareStart();

        waitForStart();

        newState(State.STATE_INITIAL);

        while(opModeIsActive()&& !isStopRequested()){
            telemetry.addData("current state: ", currentState);
            telemetry.update();
            // Execute the current state.  Each STATE's case code does the following:
            // 1: Look for an EVENT that will cause a STATE change
            // 2: If an EVENT is found, take any required ACTION, and then set the next STATE
            //   else
            // 3: If no EVENT is found, do processing for the current STATE and send TELEMETRY data for STATE.
            //
            switch (currentState) {
                case STATE_INITIAL:
                    if (mStateTime.time() > 2.0)
                    {
                        arm.setCurrentState(ArmUtil.ArmState.TRANSPORT);                 // Action: Load path to beacon
                        newState(State.STATE_DRIVE_TO_PIXEL);  // Next State:
                    }
                    else
                    {
                        // Display Diagnostic data for this state.
                        telemetry.addData("1", "In initial state");
                    }

                    break;
                case STATE_DRIVE_TO_PIXEL:
                    if (arm.getCurrentState() == ArmUtil.ArmState.IDLE) {
                        claw.openRightHand();
                        sleep(1000);
                        claw.closeRightHand();
                        sleep(1000);
                        arm.setCurrentState(ArmUtil.ArmState.LOW_SCORE);
                        newState(State.SCORE_PIXEL);
                }
                    else {
                        handleArmState();
                        telemetry.addData("2", "arm state not in idle yete");
                        telemetry.addData("current arm state: ", arm.getCurrentState());
                    }
                    break;
                case SCORE_PIXEL:
                    if (arm.getCurrentState() == ArmUtil.ArmState.IDLE && mStateTime.time() > 10.0) {
                        arm.setCurrentState(ArmUtil.ArmState.INIT);
                        newState(State.STATE_STOP);

                    } else {
                        handleArmState();
                    }
                    break;
                case STATE_STOP:
                    if (arm.getCurrentState() == ArmUtil.ArmState.IDLE) {

                    } else {
                        handleArmState();
                    }
                    break;

            }
        }

    }
    private void doArmControls() {

            arm.setCurrentState(ArmUtil.ArmState.TRANSPORT);

    }
    private void handleArmState() {
        arm.runStateMachine();
    }
    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();
        arm.init(hardwareMap);
        claw.init(hardwareMap);
    }
        //--------------------------------------------------------------------------
        //  Transition to a new state.
        //--------------------------------------------------------------------------
        private void newState(State newState)
        {
            // Reset the state time, and then change to next state.
            mStateTime.reset();
            currentState = newState;
        }
    }

