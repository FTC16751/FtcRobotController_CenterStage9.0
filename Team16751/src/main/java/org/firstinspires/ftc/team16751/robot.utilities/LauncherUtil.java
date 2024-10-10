package org.firstinspires.ftc.team16751.robot.utilities;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LauncherUtil {

    private Servo launcherServo;
    private Servo launcherAngleServo;
    private boolean launcherMoving = false;

    // Default positions
    private final double LOAD_DRONE_POSITION = 1.0;
    private final double LAUNCH_DRONE_POSITION = 0.0;
    private final double LAUNCHER_ANGLE_UP_POSITION = 0.12;
     private final double LAUNCHER_ANGLE_DOWN_POSITION = 0.0;
    private boolean toggleState,toggleLaunchAngleState;;
    public LauncherUtil(HardwareMap hardwareMap) {
        launcherServo = hardwareMap.servo.get("launcher");
        launcherServo.setPosition(LOAD_DRONE_POSITION);
        launcherAngleServo = hardwareMap.servo.get("launcherangle");
        launcherAngleServo.setPosition(LAUNCHER_ANGLE_DOWN_POSITION);
        toggleState = false;
    }

    public void raiseLauncher() {
        launcherMoving = true;
        launcherAngleServo.setPosition(LAUNCHER_ANGLE_UP_POSITION);

        while (launcherAngleServo.getPosition() != LAUNCHER_ANGLE_UP_POSITION) {
            // Wait until launcher angle servo reaches the desired position
        }
        launcherMoving = false;
    }
    public void lowerLauncherDown() {
        launcherMoving = true;
        launcherAngleServo.setPosition(LAUNCHER_ANGLE_DOWN_POSITION);
        launcherMoving = false;
    }
    public void setLauncherUp() {
        launcherMoving = true;
        launcherServo.setPosition(LOAD_DRONE_POSITION);

        while (launcherServo.getPosition() != LOAD_DRONE_POSITION) {
            // Wait until launcher angle servo reaches the desired position
        }

        launcherMoving = false;
    }

    public void setLauncherDown() {
        launcherMoving = true;
        launcherServo.setPosition(LAUNCH_DRONE_POSITION);
        launcherAngleServo.setPosition(LAUNCHER_ANGLE_DOWN_POSITION);
        launcherMoving = false;
    }

    public boolean isLauncherMoving() {
        return launcherMoving;
    }

    public double getLauncherServoPosition() {
        return launcherServo.getPosition();
    }

    public double getLauncherAngleServoPosition() {
        return launcherAngleServo.getPosition();
    }

    public void toggleServo() {
        if (!toggleState) {
            launcherServo.setPosition(LOAD_DRONE_POSITION);
        } else {
            launcherServo.setPosition(LAUNCH_DRONE_POSITION);
        }
        toggleState = !toggleState;
    }

    public void toggleLaunchAngleServo() {
        if (!toggleLaunchAngleState) {
            launcherAngleServo.setPosition(LAUNCHER_ANGLE_UP_POSITION);
        } else {
            launcherAngleServo.setPosition(LAUNCHER_ANGLE_DOWN_POSITION);
        }
        toggleLaunchAngleState = !toggleLaunchAngleState;
    }

    public boolean getToggleState() {
        return toggleState;
    }
}
