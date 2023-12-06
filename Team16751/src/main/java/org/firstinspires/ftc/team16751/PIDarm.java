package org.firstinspires.ftc.team16751;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.team16751.robot.utilities.ArmUtil;
@TeleOp
@Config

public class PIDarm extends LinearOpMode {
    public static double p=0, i=0,d=0,f=0;
    public static double goalShoulder=0;
    public static double currShoulder=0;
    public static double goalElbow = 0;
    public static double currElbow = 0;
    private PIDController shoulderController;
    private PIDController elbowController;
    private DcMotor shoulder;
    private DcMotor elbow;
    @Override
    public void runOpMode(){
        shoulder = hardwareMap.get(DcMotor.class, "Shoulder");
        shoulder.setDirection(DcMotorSimple.Direction.REVERSE);

        shoulderController = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry() );

        elbow = hardwareMap.get(DcMotor.class, "Elbow");
        elbow.setDirection(DcMotorSimple.Direction.REVERSE);
        elbowController = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while(opModeIsActive()){
            shoulderController.setPID(p,i,d);
            currShoulder = shoulder.getCurrentPosition();
            shoulder.setPower(shoulderController.calculate(currShoulder,goalShoulder));
            telemetry.addData("curr shoulder",currShoulder);
            telemetry.addData("goal shoulder",goalShoulder);


            shoulderController.setPID(p,i,d);
            currElbow = shoulder.getCurrentPosition();
            elbow.setPower(shoulderController.calculate(currElbow,goalElbow));
            telemetry.addData("curr elbow",currElbow);
            telemetry.addData("goal elbow",goalElbow);
            telemetry.update();

        }

    }
}
