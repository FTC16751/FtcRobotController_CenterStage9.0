package org.firstinspires.ftc.team16751.experiments;
// Import necessary libraries
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotArmOpMode extends LinearOpMode {
    // Define motor and servo variables
    private DcMotor shoulderMotor;
    private DcMotor elbowMotor;
    private Servo wristServo;
    private Servo leftClaw;
    private Servo rightClaw;

    // Define arm lengths and motor RPMs
    private final double SHOULDER_LENGTH = 13.25; // in inches
    private final double ELBOW_LENGTH = 15.0; // in inches
    private final double SHOULDER_RPM = 117.0;
    private final double ELBOW_RPM = 312.0;

    // Define servo positions for wrist and claws
    private double wristPosition = 0.5; // Initial position
    private double leftClawPosition = 0.0; // Initial position
    private double rightClawPosition = 1.0; // Initial position

    @Override
    public void runOpMode() {
        // Initialize hardware components
        shoulderMotor = hardwareMap.get(DcMotor.class, "shoulderMotor");
        elbowMotor = hardwareMap.get(DcMotor.class, "elbowMotor");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        // Set motor directions
        shoulderMotor.setDirection(DcMotor.Direction.FORWARD);
        elbowMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set servo initial positions
        wristServo.setPosition(wristPosition);
        leftClaw.setPosition(leftClawPosition);
        rightClaw.setPosition(rightClawPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Control shoulder and elbow using joystick axes
            double shoulderPower = -gamepad2.left_stick_y; // Invert the axis if needed
            double elbowPower = -gamepad2.right_stick_y; // Invert the axis if needed
            shoulderMotor.setPower(shoulderPower);
            elbowMotor.setPower(elbowPower);

            // Control wrist using triggers
            double wristPower = 0.0;
            if (gamepad2.left_trigger > 0) {
                wristPosition -= 0.01; // Change increment for desired movement speed
                wristPosition = Math.max(0.0, wristPosition);
            } else if (gamepad2.right_trigger > 0) {
                wristPosition += 0.01; // Change increment for desired movement speed
                wristPosition = Math.min(1.0, wristPosition);
            }
            wristServo.setPosition(wristPosition);

            // Control claws using bumpers (toggle open/close)
            if (gamepad2.left_bumper) {
                leftClawPosition = (leftClawPosition == 0.0) ? 1.0 : 0.0;
                leftClaw.setPosition(leftClawPosition);
            }
            if (gamepad2.right_bumper) {
                rightClawPosition = (rightClawPosition == 0.0) ? 1.0 : 0.0;
                rightClaw.setPosition(rightClawPosition);
            }

            telemetry.update();
        }
    }
}
