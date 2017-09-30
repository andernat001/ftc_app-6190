package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Steve on 7/12/2017.
 */

public class DriveTrainTest2 extends CyberVortexAbstract {

    public DriveTrainTest2() {
    }

    @Override
    public void init() {

        super.init();

        // Set all motors to run without encoders
        motorRightA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        super.loop();

        // Set drive motor power
        motorRightA.setPower(powerRightA);
        motorRightB.setPower(powerRightB);
        motorLeftA.setPower(powerLeftA);
        motorLeftB.setPower(powerLeftB);

        // Move Forward
        //if (gamepad1.left_stick_y > 0)
        //{




       // }

        // If the left stick and the right stick is used it halves the power  of the motors
        if (gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0 && gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0)
        {

            powerRightA = Range.clip(powerRightA, -1/2, 1/2);
            powerRightB = Range.clip(powerRightB, -1/2, 1/2);
            powerLeftA = Range.clip(powerLeftA, -1/2, 1/2);
            powerLeftB = Range.clip(powerLeftB, -1/2, 1/2);

        }
        else if (gamepad1.left_stick_x > 0 || gamepad1.left_stick_x < 0 && gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0)
        {

            powerRightA = Range.clip(powerRightA, -1/2, 1/2);
            powerRightB = Range.clip(powerRightB, -1/2, 1/2);
            powerLeftA = Range.clip(powerLeftA, -1/2, 1/2);
            powerLeftB = Range.clip(powerLeftB, -1/2, 1/2);

        }
        else
        {

            powerRightA = Range.clip(powerRightA, -1, 1);
            powerRightB = Range.clip(powerRightB, -1, 1);
            powerLeftA = Range.clip(powerLeftA, -1, 1);
            powerLeftB = Range.clip(powerLeftB, -1, 1);

        }

    }// End OpMode Loop Method

    @Override
    public void stop ()
    {
        super.stop();
    }

}// End OpMode
