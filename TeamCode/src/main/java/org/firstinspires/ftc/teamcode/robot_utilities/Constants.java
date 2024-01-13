package org.firstinspires.ftc.teamcode.robot_utilities;

public class Constants {

    // Constants specific to Autonomous programs
    public static class Autonomous {
        public static final double DESIRED_DISTANCE = 12.0;
        public static final double SPEED_GAIN = 0.02;
        public static final double STRAFE_GAIN = 0.015;
        public static final double TURN_GAIN = 0.01;
        public static final double MAX_AUTO_SPEED = 0.5;
        public static final double MAX_AUTO_STRAFE = 0.5;
        public static final double MAX_AUTO_TURN = 0.3;
    }

    // Common constants used across multiple programs
    public static class Common {
        public static final boolean USE_WEBCAM = true;
        public static final int DESIRED_TAG_ID = -1;
    }

    // Constants specific to a subsystem
    public static class Subsystem {
        public static final class Arm {
            // Constants for the Arm subsystem
        }
    }
}
