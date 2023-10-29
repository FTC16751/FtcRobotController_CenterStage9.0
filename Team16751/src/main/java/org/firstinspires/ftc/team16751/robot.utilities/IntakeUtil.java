/* P3 2021-22 season Auto Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.team16751.robot.utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class IntakeUtil {
    DcMotor intakeMotor;
    Servo intake;

    /* local OpMode members. */
    HardwareMap hardwareMap = null;

    /* Constructor */
    public IntakeUtil() {
    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;
        intake = hardwareMap.get(Servo.class, "intake");

        //Intake Motor (new bot)
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

    }

    //the following method set power to the servo based intake
    public void setIntake(int mode) {

        switch (mode) {
            case 1:
                intake.setPosition(1);
                break;
            case 2:
                intake.setPosition(-1);
                break;
            case 0:
                intake.setPosition(.5);
                break;
        }
    }

    //the following methods set controls for motor based intake
        public void stopIntake() {
            // Send calculated power to wheels
            setIntakeMotorPower(0.0);
        }//Stop

        public void setIntakeMotorPower(double v) {
            intakeMotor.setPower(v);
        }

        public double getIntakeMotorPower() {
            return intakeMotor.getPower();
        }

        public DcMotor.RunMode getIntakeMode(){
            return intakeMotor.getMode();
        }

}   //end program
