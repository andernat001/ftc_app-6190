package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Steve on 8/28/2017.
 */

public abstract class
TestAbstract extends OpMode {

    final static String
            MOTOR_TEST = "test";
    protected DcMotor
    motorTest;

    @Override
    public void init(){

        motorTest = hardwareMap.dcMotor.get(MOTOR_TEST);
        motorTest.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTest.setDirection(DcMotor.Direction.FORWARD);

    }

    public void loop(){
    }

    public void stop(){

        motorTest.setPower(0);

    }



}
