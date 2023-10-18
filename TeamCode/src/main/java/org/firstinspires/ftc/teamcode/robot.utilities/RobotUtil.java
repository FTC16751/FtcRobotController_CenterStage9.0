/* P3 2021-22 season Auto Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.teamcode.robot.utilities;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedInputStream;


public class RobotUtil {

    public ElapsedTime matchTimer;

    /* Constructor */
    public RobotUtil() {
        matchTimer = new ElapsedTime();
    }
}   //end program
