/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot_utilities.NewDriveUtil2024;
@Disabled
@Autonomous(name="Simple Auto New Class", group="Robot")

public class SimpleAutoUsingNewDriveClass extends LinearOpMode {
    NewDriveUtil2024 drive = new NewDriveUtil2024(this);
    private ElapsedTime     runtime = new ElapsedTime();
    private RevColorSensorV3 rightcolorSensor, leftcolorSensor;
    private Rev2mDistanceSensor distanceSensor;
    // Create a new ArmFeedforward with gains kS, kCos, kV, and kA
    //ArmFeedforward feedforward = new ArmFeedforward(kS, kCos, kV, kA);
    @Override
    public void runOpMode() {

        /* initialize the robot hardware */
        //drive.init(hardwareMap,telemetry);
        drive.init(hardwareMap,telemetry);
        rightcolorSensor = hardwareMap.get(RevColorSensorV3.class, "rightColorSensor");
        leftcolorSensor = hardwareMap.get(RevColorSensorV3.class, "leftColorSensor");
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        //drive.moveForwardPID(25,.25,2500);
        while (opModeIsActive()) {
            telemetry.addLine()
                    .addData("Right Color Sensor Data (RGB)", getColorSensorData(rightcolorSensor))
                    .addData("Right Color Sensor Data (HSV)", getHSVColorSensorData(rightcolorSensor))
                    .addData("Left Color Sensor Data (RGB)", getColorSensorData(leftcolorSensor))
                    .addData("Left Color Sensor Data (HSV)", getHSVColorSensorData(leftcolorSensor));
            telemetry.addData("Distance Sensor Data (inches)", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("left motor distance: ", drive.getEncoderDistance(drive.left_front_motor));
            telemetry.addData("left motor distance: ", drive.getEncoderDistance(drive.right_front_motor));
            telemetry.addData("left motor distance: ", drive.getEncoderDistance(drive.left_rear_motor));
            telemetry.addData("left motor distance: ", drive.getEncoderDistance(drive.right_rear_motor));
            telemetry.addData("Path", "Complete");
            telemetry.update();
        }


    }

    // Helper method to get color sensor data
    private String getColorSensorData(RevColorSensorV3 sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();
        return String.format("Red: %d, Green: %d, Blue: %d", red, green, blue);
    }
    private String getHSVColorSensorData(RevColorSensorV3 sensor) {
        float[] hsvValues = new float[3];
        Color.RGBToHSV(sensor.red() * 255, sensor.green() * 255, sensor.blue() * 255, hsvValues);
        return String.format("Hue: %.2f, Saturation: %.2f, Value: %.2f", hsvValues[0], hsvValues[1], hsvValues[2]);
    }
}
