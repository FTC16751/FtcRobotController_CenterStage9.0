package org.firstinspires.ftc.teamcode.TeleOp.Experiments;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp
@Config
@Disabled
public class PIDdemo extends LinearOpMode {
    public static double p=0, i=0,d=0,f=0;
    public static double goal=0;
    public static double curr=0;
    private PIDController slideController;
    private DcMotor slide;
    @Override
    public void runOpMode(){
        slide = hardwareMap.get(DcMotor.class, "Slide");
        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        slideController = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry() );
        waitForStart();
        while(opModeIsActive()){
            slideController.setPID(p,i,d);
            curr = slide.getCurrentPosition();
            slide.setPower(slideController.calculate(curr,goal));
            telemetry.addData("curr",curr);
            telemetry.addData("goal",goal);
            telemetry.update();

        }

    }
}
