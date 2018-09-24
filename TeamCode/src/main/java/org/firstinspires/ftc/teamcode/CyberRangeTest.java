package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Steve on 10/9/2017.
 */

public class CyberRangeTest extends OpMode
{
    double varname;

    protected ModernRoboticsI2cRangeSensor
            rangeSensor;

    @Override
    public void init()
    {
    }

    public void loop()
    {
        if (rangeSensor.cmOptical() <= 3.0)
        {
            varname = rangeSensor.cmOptical();
        }
        else
        {
            varname = rangeSensor.cmUltrasonic();
        }
    }

}
