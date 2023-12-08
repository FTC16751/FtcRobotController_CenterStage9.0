package org.firstinspires.ftc.teamcode.robot.utilities;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LauncherUtil {
    private Servo launcherServo;
    private Servo launcherAngleServo;
    private boolean launcherMoving = false;

    // Default positions
    private final double LAUNCHER_UP_POSITION = 0.2;
    private final double LAUNCHER_DOWN_POSITION = 0.6;
    private final double LAUNCHER_ANGLE_UP_POSITION = 0.1;
    private final double LAUNCHER_ANGLE_DOWN_POSITION = 0.0;

    public LauncherUtil(HardwareMap hardwareMap) {
        launcherServo = hardwareMap.servo.get("launcher");
        launcherServo.setPosition(LAUNCHER_DOWN_POSITION);
        launcherAngleServo = hardwareMap.servo.get("launcherangle");
        launcherAngleServo.setPosition(LAUNCHER_ANGLE_DOWN_POSITION);
    }

    public void raiseLauncher() {
        launcherMoving = true;
        launcherAngleServo.setPosition(LAUNCHER_ANGLE_UP_POSITION);

        while (launcherAngleServo.getPosition() != LAUNCHER_ANGLE_UP_POSITION) {
            // Wait until launcher angle servo reaches the desired position
        }
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
}
