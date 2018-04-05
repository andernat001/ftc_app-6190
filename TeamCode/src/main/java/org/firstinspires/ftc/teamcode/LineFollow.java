package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.LineFollow.direction.left;
import static org.firstinspires.ftc.teamcode.LineFollow.direction.right;

@Autonomous(name = "LineFollow", group = "RiderModes")
public class LineFollow extends CyberRelicAbstract {

    DcMotor
            rightDrive,leftDrive;

    ColorSensor
            colorSensor;

    direction movementDirection = left;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);  //Added so opMode does not sleep

    @Override
    public void init() {

        super.init();

        colorSensor.enableLed(true);

    }

    @Override
    public void loop() {

        int colorValue;
        colorValue = colorSensor.alpha();

        if(colorValue > .100 && movementDirection == left){
            if(timer.milliseconds() < 250) {
                rightDrive.setPower(.4);
                leftDrive.setPower(.2);
            }
            if(timer.milliseconds() > 250 && timer.milliseconds() < 300){
                rightDrive.setPower(0);
                leftDrive.setPower(0);
            }
            if(timer.milliseconds() > 300) {
                rightDrive.setPower(.4);
                leftDrive.setPower(.4);
            }
        }
        if(colorValue < .100 && movementDirection == left){
            movementDirection = right;
        }
        if(colorValue > .100 && movementDirection == right){
            if(timer.milliseconds() < 250) {
                rightDrive.setPower(.2);
                leftDrive.setPower(.4);
            }
            if(timer.milliseconds() > 250 && timer.milliseconds() < 300){
                rightDrive.setPower(0);
                leftDrive.setPower(0);
            }
            if(timer.milliseconds() > 300) {
                rightDrive.setPower(.4);
                leftDrive.setPower(.4);
            }
        }
        if(colorValue < .100 && movementDirection == right){
            movementDirection = left;
        }
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