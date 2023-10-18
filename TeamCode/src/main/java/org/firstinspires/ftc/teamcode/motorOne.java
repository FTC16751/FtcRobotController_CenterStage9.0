package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** Configuration file
 * * control hub:
 * motor port 00: motorOne
 *
 */

@TeleOp(group ="examples")

public class motorOne extends LinearOpMode {

    private DcMotor motorOne;
    private double motorOneZeroPower = 0.0;
    private double motorOnePower = 1.0;

    @Override
    public void runOpMode() throws InterruptedException{
        initHardware();
        while(!isStarted()) {
            motorTelemetry();
        }
        waitForStart();
        while(opModeIsActive()) {
            teleOpControls();
            motorTelemetry();
        }

    }

    private void teleOpControls() {
        if (gamepad1.x) {
            motorOne.setPower(-motorOnePower);
        }
        if (gamepad1.a) {
            motorOne.setPower(motorOnePower);
        }
        if (gamepad1.b) {
            motorOne.setPower(motorOnePower);
        }
    }

    private void motorTelemetry() {
    }

    public void initHardware() {
        initMotorOne();
    }

    public void initMotorOne(){
        motorOne = hardwareMap.get(DcMotor.class, "motorOne");
        motorOne.setDirection(DcMotorSimple.Direction.FORWARD);
        motorOne.setPower(motorOneZeroPower);
        motorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
