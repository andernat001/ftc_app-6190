package org.firstinspires.ftc.teamcode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TestProgram")
public class NewElectronicsTest extends TestAbstract {

    @Override
    public void init() {
        super.init();

        motorTest.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {

        super.loop();

        if (gamepad1.a)
        {

            motorTest.setPower(1);

        }
        else
        {
            motorTest.setPower(0);
        }

    }
        @Override
        public void stop ()
        {
            super.stop();
        }



}
