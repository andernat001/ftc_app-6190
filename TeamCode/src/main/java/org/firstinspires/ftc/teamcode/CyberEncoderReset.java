package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Encoder Reset", group = "RiderModes")
public class CyberEncoderReset extends CyberRoverAbstract {


    //------------------------------------------------------------------
    // Robot OpMode Loop Method
    //------------------------------------------------------------------

    @Override
    public void init() {

        super.init();

    }

    @Override
    public void loop() {

        super.loop();

        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // End Robot Sequence
    }
}