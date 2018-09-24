package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Calendar;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.LineFollow.direction.left;
import static org.firstinspires.ftc.teamcode.LineFollow.direction.right;

@Autonomous(name = "LineFollow", group = "RiderModes")
public class LineFollow extends CyberRoverAbstract {

    Calendar startTime, loopTime;

    DcMotor
            rightDrive,leftDrive;

    ColorSensor
            colorSensor;

    int
        blackThreshold = 0, whiteThreshold = 0;

    long loopIterations = 1;

    direction movementDirection = left;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);  //Added so opMode does not sleep

    @Override
    public void init() {

        super.init();

        colorSensor.enableLed(true);

    }

    @Override
    public void start() {
        super.start();

        startTime = Calendar.getInstance();

    }

    @Override
    public void loop() {
        loopTime = Calendar.getInstance();

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
        if (loopIterations % 500 == 0)
            telemetry.addData("Loop Time", (Calendar.getInstance().getTimeInMillis() - loopTime.getTimeInMillis()) + " ms");
            telemetry.addData("Running for", ((Calendar.getInstance().getTimeInMillis() - startTime.getTimeInMillis()) / 1000) + " seconds");

        loopIterations++;
    }

    private void slp(int slptime) {
        try {
            sleep(slptime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    enum direction {left, right}
}