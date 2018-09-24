package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Steve on 7/3/2018.
 */

public class MotorLimit
{

    // MotorLimit method - Recommend not using this method
    // This method prevents over-extended motor movement. Once a limit is reached, you cannot go
    // any further, but you may reverse course. Unfortunately this does not prevent significant
    // overshoot. A better way to limit motor distance is to place the motor into Run-To-Position
    // mode, and then adjust power manually.
    //
    // Method Parameters:
    //     powerValue = desired motor throttle value
    //     lower limit / upper limit = allowed range of movementmotor.getPower(), motor.getCurrentPosition()
    //
    //  Motor Output:
    //      powerValue = recalculated motor throttle

    private int lowerLimit, upperLimit;

    public MotorLimit(int lowerLimit, int upperLimit)
    {
        this.lowerLimit = lowerLimit;
        this.upperLimit = upperLimit;
    }

    public double limit(double powerValue, int currentPos)
    {
        if (currentPos > upperLimit && powerValue > 0)
        {
            powerValue = 0;
        }

        if (currentPos < lowerLimit && powerValue < 0)
        {
            powerValue = 0;
        }

        return powerValue;
    }

    public double limit(DcMotor motor)
    {
        return limit(motor.getPower(), motor.getCurrentPosition());
    }

    void test()
    {
        DcMotor motor = null;
        MotorLimit liftLimit = new MotorLimit(0, 300);

        motor.setPower(45327905);
        motor.setPower(liftLimit.limit(motor));
    }
}
