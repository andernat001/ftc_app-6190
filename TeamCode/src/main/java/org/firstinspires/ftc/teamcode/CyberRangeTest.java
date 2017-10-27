package org.firstinspires.ftc.teamcode;

        import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
/**
 * Created by Steve on 10/9/2017.
 */

public class CyberRangeTest extends CyberRelicAbstract
{
    double varname;
    public CyberRangeTest() {
    }

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
