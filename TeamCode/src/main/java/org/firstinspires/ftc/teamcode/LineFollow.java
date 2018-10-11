package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.LineFollow.direction.left;
import static org.firstinspires.ftc.teamcode.LineFollow.direction.right;

@Autonomous(name = "LineFollow", group = "RiderModes")
public class LineFollow extends OpMode {

    DcMotor
            rightDrive,leftDrive;

    ColorSensor
            colorSensor;

    int
        blackThreshold = 0, whiteThreshold = 0;


    direction movementDirection = left;

    @Override
    public void init() {

        colorSensor.enableLed(true);

    }

    @Override
    public void loop() {

        int colorValue;
        colorValue = colorSensor.alpha();

        switch (movementDirection) {
            case left:
                if (colorValue < whiteThreshold) {
                    rightDrive.setPower(0.4);
                    leftDrive.setPower(0.3);
                    movementDirection = right;
                }

            case right:
                if (colorValue > blackThreshold) {
                    rightDrive.setPower(0.3);
                    leftDrive.setPower(0.4);
                    movementDirection = left;
                }

        }
    }

    enum direction {left, right}
}