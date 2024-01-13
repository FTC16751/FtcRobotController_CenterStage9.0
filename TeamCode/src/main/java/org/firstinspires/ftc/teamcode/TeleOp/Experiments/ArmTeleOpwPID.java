package org.firstinspires.ftc.teamcode.TeleOp.Experiments;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.controller.PIDController;
@TeleOp(name = "ArmTeleOpwPID", group = "Tests")

public class ArmTeleOpwPID extends LinearOpMode {

    private DcMotor shoulderLeft, shoulderRight, elbowLeft, elbowRight;
    private final int[] startPositions = {0, 0, 0, 0};
    private final int[] levelPositions = {175 ,0,0,0};
    private final int[] scorePositions = {1228,0, 0, 0};
    private final int[] scoreHiohPositions = {0, 0, 0, 0};
    private int currentState = 0;
    private int[] targetPositions = startPositions;

    public static double p=0.002, i=0.1,d=0.0001,f=0.1;
    private PIDController shoulderLeftController ,shoulderRightController ,elbowLeftController,elbowRightController;
    public static double TICKS_PER_REV = 3895.9 ;
    public static double TOTAL_DEGREES = 360 ;
    public static double Kv = 0;
    public static double Ka = 0;
    public static double Ks = 0;
    // Kg and Kcos should NOT be used together, one or the other should be zero.
    public static double velo = 0;
    public static double accel = 0;
    public static double Kcos = 0.1;
    ArmFeedforward feedforward;// = new ArmFeedforward(Ks, Kcos, Kv, Ka);

    @Override
    public void runOpMode() {
        shoulderLeft = hardwareMap.get(DcMotor.class, "shoulderLeft");
        shoulderRight = hardwareMap.get(DcMotor.class, "shoulderRight");
        elbowLeft = hardwareMap.get(DcMotor.class, "elbowLeft");
        elbowRight = hardwareMap.get(DcMotor.class, "elbowRight");

        shoulderLeftController = new PIDController(p,i,d);
        shoulderRightController = new PIDController(p,i,d);
        elbowLeftController = new PIDController(p,i,d);
        elbowRightController = new PIDController(p,i,d);
// Create a new ArmFeedforward with gains kS, kCos, kV, and kA
        feedforward = new ArmFeedforward(Ks, Kcos, Kv, Ka);
        shoulderLeft.setDirection(DcMotor.Direction.REVERSE);
        shoulderRight.setDirection(DcMotor.Direction.FORWARD);
        elbowLeft.setDirection(DcMotor.Direction.FORWARD);
        elbowRight.setDirection(DcMotor.Direction.FORWARD);

        shoulderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoulderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbowLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbowRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            //ArmFeedforward feedforward = new ArmFeedforward(Ks, Kcos, Kv, Ka);
            doTelemetry();
            if (gamepad2.a && currentState != 0) {
                targetPositions = startPositions;
                currentState = 0;
                moveArmToPosition(targetPositions);

            } else if (gamepad2.b && currentState != 1) {
                targetPositions = levelPositions;
                currentState = 1;
                moveArmToPosition(targetPositions);


            } else if (gamepad2.x && currentState != 2) {
                targetPositions = scorePositions;
                currentState = 2;
                moveArmToPosition(targetPositions);
            } else if (gamepad2.y && currentState != 3) {
                targetPositions = scoreHiohPositions;
                currentState = 3;
                moveArmToPosition(targetPositions);
            }


        }
    }

    private void doTelemetry() {
        telemetry.addData("left shoulder: ", shoulderLeft.getCurrentPosition());
        telemetry.addData("right shoulder: ", shoulderRight.getCurrentPosition());
        telemetry.addData("left elbow: ", elbowLeft.getCurrentPosition());
        telemetry.addData("right elbow: ", elbowRight.getCurrentPosition());
        telemetry.update();
    }

    private void moveArmToPosition(int[] positions) {
        telemetry.addLine("Move Arm Data");
        telemetry.addData("move to position:", "move");
        shoulderLeft.setPower(feedforward.calculate(Math.toRadians(90), velo , accel));
        /*
        shoulderLeftController.setPID(p,i,d);
        shoulderLeft.setPower(shoulderLeftController.calculate(shoulderLeft.getCurrentPosition(),positions[0]));
telemetry.addData("set power: ", shoulderLeftController.calculate(shoulderLeft.getCurrentPosition(),positions[0]));
//telemetry.update();

        shoulderRightController.setPID(p,i,d);
        shoulderRight.setPower(shoulderRightController.calculate(shoulderRight.getCurrentPosition(),positions[1]));

        elbowLeftController.setPID(p,i,d);
        elbowLeft.setPower(elbowLeftController.calculate(elbowLeft.getCurrentPosition(),positions[2]));

        elbowRightController.setPID(p,i,d);
        elbowRight.setPower(elbowRightController.calculate(elbowRight.getCurrentPosition(),positions[3]));

         */
    }
    public static int degreesToEncoderTicks(double degrees)
    {
        return (int)Math.round((TICKS_PER_REV ) / (TOTAL_DEGREES / degrees));
    }

    public static double encoderTicksToDegrees(int ticks)
    {
        return (TOTAL_DEGREES * ticks) / TICKS_PER_REV;
    }

}


