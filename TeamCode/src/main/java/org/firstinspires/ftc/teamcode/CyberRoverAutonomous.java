package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static java.lang.Thread.sleep;

@Autonomous(name = "Rover", group = "RiderModes")
public class CyberRoverAutonomous extends CyberRoverAbstract{

    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);  //Added so opMode does not sleep

    //OpenGLMatrix lastLocation = null;


    //VuforiaLocalizer vuforia;
    //BNO055IMU imu;
    @Override
    public void init() {

        super.init();

    }

    //Added start to reset encoders when Play is pressed
    @Override
    public void start() {
        motorRightA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    @Override
    public void loop() {

        switch (seqRobot) {

            case 1:

                seqRobot++;
                break;


            default:

                break;

        }
    }

    @Override
    public void stop() {
        motorLeftA.setPower(0);
        motorLeftB.setPower(0);
        motorRightA.setPower(0);
        motorRightB.setPower(0);
    }

    private void slp(int slptime) {
        try {
            sleep(slptime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}