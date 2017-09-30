package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.Range;
        import com.qualcomm.robotcore.hardware.GyroSensor;

@TeleOp(name = "Mecanum`.0Drive")
public class DriveTrainTest extends CyberRelicAbstract {

    public DriveTrainTest() {
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

        // Set controls
        velocityDrive = -gamepad1.left_stick_y;
        strafeDrive = gamepad1.left_stick_x;
        rotationDrive = gamepad1.right_stick_x;

        // Set algorithm
        powerRightA = velocityDrive + rotationDrive + strafeDrive;
        powerRightB = velocityDrive + rotationDrive - strafeDrive;
        powerLeftA = velocityDrive - rotationDrive - strafeDrive;
        powerLeftB = velocityDrive - rotationDrive + strafeDrive;

        // Scale drive motor power for better control at low power
        powerRightA = (float) scaleInput(powerRightA);
        powerRightB = (float) scaleInput(powerRightB);
        powerLeftA = (float) scaleInput(powerLeftA);
        powerLeftB = (float) scaleInput(powerLeftB);

        // Round the joystick's value when between 0.05 and -0.05 to 0
        if (gamepad1.left_stick_x <= 0.05 && gamepad1.left_stick_x >= -0.05)
        {
            gamepad1.left_stick_x = 0;
        }

        if (gamepad1.left_stick_y <= 0.05 && gamepad1.left_stick_y >= -0.05)
        {
            gamepad1.left_stick_y = 0;
        }

        if (gamepad1.right_stick_x <= 0.05 && gamepad1.right_stick_x >= -0.05)
        {
            gamepad1.right_stick_x = 0;
        }

        // If the left stick and the right stick is used it halves the power  of the motors
        if (gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0 && gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0)
        {


            powerRightA = Range.clip(powerRightA, -0.5f, 0.5f);
            powerRightB = Range.clip(powerRightB, -0.5f, 0.5f);
            powerLeftA = Range.clip(powerLeftA, -0.5f, 0.5f);
            powerLeftB = Range.clip(powerLeftB, -0.5f, 0.5f);

        }
        else if (gamepad1.left_stick_x > 0 || gamepad1.left_stick_x < 0 && gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0)
        {

            powerRightA = Range.clip(powerRightA, -0.5f, 0.5f);
            powerRightB = Range.clip(powerRightB, -0.5f, 0.5f);
            powerLeftA = Range.clip(powerLeftA, -0.5f, 0.5f);
            powerLeftB = Range.clip(powerLeftB, -0.5f, 0.5f);

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


