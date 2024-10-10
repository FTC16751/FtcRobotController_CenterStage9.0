package org.firstinspires.ftc.team16751BigBot.experiments;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TouchTest")
public class TouchTest extends LinearOpMode {
    // Define variables for our touch sensor and motor
    TouchSensor touch;
    TouchSensor touch2;



    @Override
    public void runOpMode() {
        // Get the touch sensor and motor from hardwareMap
        touch = hardwareMap.get(TouchSensor.class, "lmagswitch");
        touch2 = hardwareMap.get(TouchSensor.class, "rmagswitch");

        // Wait for the play button to be pressed
        waitForStart();

        // Loop while the Op Mode is running
        while (opModeIsActive()) {
            // If the Magnetic Limit Swtch is pressed, stop the motor
                telemetry.addData("left is pressed",touch.isPressed());
                telemetry.addData("right is pressed", touch2.isPressed());
                telemetry.update();
            }


        }
    }




