package org.firstinspires.ftc.teamcode.TeleOp.Experiments;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ArmTeleOp", group = "Tests")
@Disabled
public class ArmTeleOp extends LinearOpMode {

    private DcMotor shoulderLeft, shoulderRight, elbowLeft, elbowRight;
    private final int[] startPositions = {0, 0, 0, 0};
    private final int[] levelPositions = {175 ,251 ,20 ,5};
    private final int[] scorePositions = {1228,1258, 0, 0};
    private final int[] scoreHiohPositions = {0, 0, 0, 0};
    private int currentState = 0;
    private int[] targetPositions = startPositions;

    @Override
    public void runOpMode() {
        shoulderLeft = hardwareMap.get(DcMotor.class, "shoulderLeft");
        shoulderRight = hardwareMap.get(DcMotor.class, "shoulderRight");
        elbowLeft = hardwareMap.get(DcMotor.class, "elbowLeft");
        elbowRight = hardwareMap.get(DcMotor.class, "elbowRight");

        shoulderLeft.setDirection(DcMotor.Direction.REVERSE);
        shoulderRight.setDirection(DcMotor.Direction.FORWARD);
        elbowLeft.setDirection(DcMotor.Direction.FORWARD);
        elbowRight.setDirection(DcMotor.Direction.FORWARD);

        shoulderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoulderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        while (opModeIsActive()) {

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

            telemetry.addData("left shoulder: ", shoulderLeft.getCurrentPosition());
            telemetry.addData("right shoulder: ", shoulderRight.getCurrentPosition());
            telemetry.addData("left elbow: ", elbowLeft.getCurrentPosition());
            telemetry.addData("right elbow: ", elbowRight.getCurrentPosition());
            telemetry.update();
        }
    }

    private void moveArmToPosition(int[] positions) {
        shoulderLeft.setTargetPosition(positions[0]);
        shoulderRight.setTargetPosition(positions[1]);
        elbowLeft.setTargetPosition(positions[2]);
        elbowRight.setTargetPosition(positions[3]);

        shoulderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shoulderLeft.setPower(1.0);
        shoulderRight.setPower(1.0);
        elbowLeft.setPower(.5);
        elbowRight.setPower(.5);



        //stopMotors();
    }

    private boolean isMoving() {
        return shoulderLeft.isBusy() || shoulderRight.isBusy() || elbowLeft.isBusy() || elbowRight.isBusy();
    }

    private void stopMotors() {
        shoulderLeft.setPower(0);
        shoulderRight.setPower(0);
        elbowLeft.setPower(0);
        elbowRight.setPower(0);

        shoulderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}


