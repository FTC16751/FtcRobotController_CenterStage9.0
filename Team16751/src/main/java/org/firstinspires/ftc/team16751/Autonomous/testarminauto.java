package org.firstinspires.ftc.team16751.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16751.robot.utilities.ArmUtil;
import org.firstinspires.ftc.team16751.robot.utilities.ClawUtil;
import org.firstinspires.ftc.team16751.robot.utilities.DriveUtil2023;
import org.firstinspires.ftc.team16751.robot.utilities.TeamElementSubsystem;

@Autonomous(name="Test Arm in AUto", group="Auto")

public class testarminauto extends LinearOpMode {
    private enum State
    {
        STATE_INITIAL,
        STATE_DRIVE_TO_SPIKE_MARK,
        STATE_DRIVE_TO_BACKDROP,
        STATE_SCORE_PIXEL,
        STATE_PARK,
        STATE_STOP,
    }
    public int element_zone = 1;

    private TeamElementSubsystem teamElementDetection = null;

    boolean togglePreview = true;
    private DriveUtil2023 driveUtil2023 = null;
    private State       currentState;    // Current State Machine State.
    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state
    ArmUtil arm = new ArmUtil(this);
    ClawUtil claw = new ClawUtil(this);
    ArmUtil.ArmState currentArmState;
    String curAlliance;
    //Initializing Hardware


//Output on driver control hub with element zone, camera stream, and alliance color
    public void runOpMode() {
        HardwareStart();
        curAlliance = "blue";

        while (!opModeIsActive() && !isStopRequested()) {
            //doVisionSetup();
            element_zone = teamElementDetection.elementDetection(telemetry);
            if (togglePreview && gamepad2.a) {
                togglePreview = false;
                teamElementDetection.toggleAverageZone(gamepad2);
            } else if (!gamepad2.a) {
                togglePreview = true;
            }


            if (gamepad1.x) {
                curAlliance = "blue";
            } else if (gamepad1.b) {
                curAlliance = "red";
            }
            teamElementDetection.setAlliance(curAlliance);
            telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
            telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());
            telemetry.update();
            telemetry.addData("Object", "Passed waitForStart");
            telemetry.update();
        }

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
                    if (mStateTime.time() > 1.0)
                    {
                        arm.setCurrentState(ArmUtil.ArmState.TRANSPORT);                 // Action: Load path to beacon
                        newState(State.STATE_DRIVE_TO_SPIKE_MARK);  // Next State:
                    }
                    else
                    {
                        // Display Diagnostic data for this state.
                        telemetry.addData("1", "In initial state");
                    }

                    break;
                case STATE_DRIVE_TO_SPIKE_MARK:
                    if (mStateTime.time() > 5.0) {
                        newState(State.STATE_DRIVE_TO_BACKDROP);
                }
                    else {
                        handleArmState();
                        telemetry.addData("2", "arm state not in idle yete");
                        telemetry.addData("current arm state: ", arm.getCurrentState());
                        if(element_zone==1&&curAlliance=="blue"){
                            driveUtil2023.driveRobotDistanceForward(73,0.5);
                            driveUtil2023.rotateLeft90Degrees();
                            claw.openRightHand();
                        }
                    }
                    break;
                case STATE_DRIVE_TO_BACKDROP:
                    if (mStateTime.time() > 10.0) {
                        arm.setCurrentState(ArmUtil.ArmState.INIT);
                        newState(State.STATE_STOP);

                    } else {
                        handleArmState();
                        driveUtil2023.rotateRight90Degrees();

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

    private void doVisionSetup() {


    }

    private void doArmControls() {

            arm.setCurrentState(ArmUtil.ArmState.TRANSPORT);

    }
    private void handleArmState() {
        arm.runStateMachine();
    }
    public void HardwareStart() {
        teamElementDetection = new TeamElementSubsystem(hardwareMap);

        driveUtil2023 = new DriveUtil2023(this);
        driveUtil2023.init(hardwareMap);
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

