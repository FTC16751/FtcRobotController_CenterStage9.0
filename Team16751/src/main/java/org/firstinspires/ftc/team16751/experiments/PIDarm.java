package org.firstinspires.ftc.team16751.experiments;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="PID Arm Test", group="Teleop")
@Disabled
@Config

public class PIDarm extends LinearOpMode {
    //public static double p=0.002, i=0.1,d=0.0001,f=0.1;
    public static double p=0.002, i=0.03,d=0.0001,f=0.1;
    public static double goalShoulder;
    public static double goalElbow;
    public double currShoulder;
    public double currElbow;
    private PIDController shoulderLeftController,shoulderRightController;
    private PIDController elbowLeftController, elbowRightController;
    private DcMotor shoulderLeft,shoulderRight;
    private DcMotor elbowLeft, elbowRight;
   // ArmUtil arm = new ArmUtil(this);
    @Override
    public void runOpMode(){
        //shoulder = hardwareMap.get(DcMotor.class, "Shoulder");
        shoulderLeft = hardwareMap.get(DcMotor.class, "shoulderLeft");
        shoulderLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shoulderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulderLeftController = new PIDController(p,i,d);

        shoulderRight = hardwareMap.get(DcMotor.class, "shoulderRight");
        shoulderRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shoulderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulderRightController = new PIDController(p,i,d);

        //elbow = hardwareMap.get(DcMotor.class, "Elbow");
        elbowLeft = hardwareMap.get(DcMotor.class, "elbowLeft");
        elbowLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        elbowLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbowLeftController = new PIDController(p,i,d);

        elbowRight = hardwareMap.get(DcMotor.class, "elbowRight");
        elbowRight.setDirection(DcMotorSimple.Direction.REVERSE);
        elbowRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbowRightController = new PIDController(p,i,d);

        waitForStart();

        while(opModeIsActive()){
            shoulderLeftController.setPID(p,i,d);
            shoulderRightController.setPID(p,i,d);
            currShoulder = shoulderRight.getCurrentPosition();
            shoulderLeft.setPower(shoulderLeftController.calculate(shoulderLeft.getCurrentPosition(),goalShoulder));
            shoulderRight.setPower(shoulderRightController.calculate(shoulderRight.getCurrentPosition(),(goalShoulder*1.05)));

            elbowLeftController.setPID(p,i,d);
            elbowLeft.setPower(elbowLeftController.calculate(elbowLeft.getCurrentPosition(),goalElbow));

            elbowRightController.setPID(p,i,d);
            elbowRight.setPower(elbowRightController.calculate(elbowRight.getCurrentPosition(),goalElbow));

            doTelemetry();

        }


    }

    private void doTelemetry() {
        telemetry.addData("left shoulder position",shoulderLeft.getCurrentPosition());
        telemetry.addData("right shoulder position",shoulderRight.getCurrentPosition());
        telemetry.addData("goal shoulder",goalShoulder);
        telemetry.addData("shoulder power", shoulderRightController.calculate(currShoulder,goalShoulder));
        telemetry.addData("left elbow position",elbowLeft.getCurrentPosition());
        telemetry.addData("right elbow position",elbowRight.getCurrentPosition());
        telemetry.addData("goal elbow",goalElbow);
        telemetry.update();
    }
}
