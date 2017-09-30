package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Steve on 9/18/2017.
 */
@Autonomous(name = "Lol I'm A Test :D")
public class LolImATest extends LinearOpMode {

    DcMotor motorLeftA;
    DcMotor motorLeftB;
    DcMotor motorRightA;
    DcMotor motorRightB;

    @Override
    public void runOpMode() throws InterruptedException {

        // init

        motorLeftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart(); // waits for start button

        // auto stuff
        motorLeftA.setPower(1);
        wait(1000);
        motorLeftA.setPower(0);

    }
}
