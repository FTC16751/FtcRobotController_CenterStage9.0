package org.firstinspires.ftc.team16751.roadrunner;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team16751.robot.utilities.ArmUtil;
import org.firstinspires.ftc.team16751.robot.utilities.ClawUtil;
import org.firstinspires.ftc.team16751.robot.utilities.NewDriveUtil2024;
import org.firstinspires.ftc.team16751.robot.utilities.TeamElementSubsystem;

@Autonomous(name="RRvAuto w Vision Backstage", group="Auto")
@Disabled
public class RRAutoVision extends LinearOpMode {


    public int element_zone = 1;

    private TeamElementSubsystem teamElementDetection = null;

    boolean togglePreview = true;
    private NewDriveUtil2024 drive = null;
    ArmUtil arm = new ArmUtil(this);
    ClawUtil claw = new ClawUtil(this);
    //Initializing Hardware
    public void HardwareStart() {
        teamElementDetection = new TeamElementSubsystem(hardwareMap);
        arm.init(hardwareMap);
        claw.init(hardwareMap);
        claw.setClawClosed();
        arm.setWristPosition(.45);
    }

    //Output on driver control hub with element zone, camera stream, and alliance color
    public void runOpMode() {

        HardwareStart();

    }


}

