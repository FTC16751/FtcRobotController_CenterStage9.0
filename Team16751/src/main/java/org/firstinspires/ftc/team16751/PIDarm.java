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
@TeleOp(name="PID Arm Test", group="Teleop")

@Config

public class PIDarm extends LinearOpMode {
    public static double p=0.002, i=0.1,d=0.0001,f=0.1;
    public static double goalShoulder;
    public double currShoulder;
    public double goalElbow;
    public double currElbow;
    private PIDController shoulderController;
    private PIDController elbowController;
    private DcMotor shoulder;
    private DcMotor elbow;
    ArmUtil arm = new ArmUtil(this);
    @Override
    public void runOpMode(){
        shoulder = hardwareMap.get(DcMotor.class, "Shoulder");
        shoulder.setDirection(DcMotorSimple.Direction.FORWARD);

        shoulderController = new PIDController(p,i,d);

        elbow = hardwareMap.get(DcMotor.class, "Elbow");
        elbow.setDirection(DcMotorSimple.Direction.FORWARD);
        elbowController = new PIDController(p,i,d);

        waitForStart();
        while(opModeIsActive()){
            shoulderController.setPID(p,i,d);
            currShoulder = shoulder.getCurrentPosition();
            shoulder.setPower(shoulderController.calculate(currShoulder,goalShoulder));
            telemetry.addData("curr shoulder",currShoulder);
            telemetry.addData("goal shoulder",goalShoulder);
            telemetry.addData("shoulder power",shoulderController.calculate(currShoulder,goalShoulder));
            //Shoulder

            elbowController.setPID(p,i,d);
            currElbow = elbow.getCurrentPosition();
            elbow.setPower(elbowController.calculate(currElbow,goalElbow));
            telemetry.addData("curr elbow",currElbow);
            telemetry.addData("goal elbow",goalElbow);
            telemetry.update();

        }


    }
}
