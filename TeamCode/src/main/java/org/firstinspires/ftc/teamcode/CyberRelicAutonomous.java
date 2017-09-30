package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.hardware.GyroSensor;

    import static java.lang.Thread.sleep;

@Autonomous(name = "RelicAuto", group = "RiderModes")
public class CyberRelicAutonomous extends CyberRelicAbsrtact {

    @Override
    public void init() {

        super.init();


    }

    @Override
    public void loop()
    {

        super.loop();

        switch (seqRobot)
        {

            case 1:
            {
                motorLeftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                seqRobot++;
                break;
            }











            case 99:  // Done
            {
                break;
            }

            default:
            {
                break;
            }
        }






    }
}
