package org.firstinspires.ftc.team16751.robot.utilities;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LauncherUtil {
    public static final double LAUNCHER_DOWN_POSITION = 0.25;
    public static final double LAUNCHER_ANGLE_DOWN_POSITION = 0.00;
    private Servo launcherServo;
    private Servo launcherAngleServo;
    private boolean launcherMoving = false;

    // Default positions
    private final double LAUNCHER_UP_POSITION = 0.0;
    //private final double LAUNCHER_DOWN_POSITION = 0.35;
    private final double LAUNCHER_ANGLE_UP_POSITION = 0.1;
    // private final double LAUNCHER_ANGLE_DOWN_POSITION = 0.0;
    private boolean toggleState;
    public LauncherUtil(HardwareMap hardwareMap) {
        launcherServo = hardwareMap.servo.get("launcher");
        launcherServo.setPosition(LAUNCHER_DOWN_POSITION);
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
        launcherServo.setPosition(LAUNCHER_UP_POSITION);

        while (launcherServo.getPosition() != LAUNCHER_UP_POSITION) {
            // Wait until launcher angle servo reaches the desired position
        }

        launcherMoving = false;
    }

    public void setLauncherDown() {
        launcherMoving = true;
        launcherServo.setPosition(LAUNCHER_DOWN_POSITION);
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
            launcherServo.setPosition(LAUNCHER_UP_POSITION);
        } else {
            launcherServo.setPosition(LAUNCHER_DOWN_POSITION);
        }
        toggleState = !toggleState;
    }

    public boolean getToggleState() {
        return toggleState;
    }
}
