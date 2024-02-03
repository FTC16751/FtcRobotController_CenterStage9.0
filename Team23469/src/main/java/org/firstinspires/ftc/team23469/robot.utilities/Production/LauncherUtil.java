package org.firstinspires.ftc.team23469.robot.utilities.Production;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LauncherUtil {
    private Servo launcherServo;
    private Servo launcherAngleServo;

    private boolean launcherMoving = false;

    // Default positions
    private final double LAUNCHER_TRIGGER_CLOSED_POSITION = 0.00;
    private final double LAUNCHER_TRIGGER_OPEN_POSITION = 1.0;
    private final double LAUNCHER_ANGLE_UP_POSITION = 0.1;
    private final double LAUNCHER_ANGLE_DOWN_POSITION = 0.0;

    public LauncherUtil(HardwareMap hardwareMap) {
        launcherServo = hardwareMap.servo.get("launcher");
        launcherServo.setPosition(LAUNCHER_TRIGGER_CLOSED_POSITION);
        launcherAngleServo = hardwareMap.servo.get("launcherangle");
        launcherAngleServo.setPosition(LAUNCHER_ANGLE_DOWN_POSITION);
    }

    public void raiseLauncher() {
        launcherMoving = true;
        launcherAngleServo.setPosition(LAUNCHER_ANGLE_UP_POSITION);
        launcherMoving = false;
    }
    public void launchTriggerClose() {
        launcherMoving = true;
        launcherServo.setPosition(LAUNCHER_TRIGGER_CLOSED_POSITION);
        launcherMoving = false;
    }

    public void launchTriggerOpen() {
        launcherMoving = true;
        launcherServo.setPosition(LAUNCHER_TRIGGER_OPEN_POSITION);
        launcherMoving = false;
    }

    public boolean isLauncherMoving() {
        return launcherMoving;
    }

}
