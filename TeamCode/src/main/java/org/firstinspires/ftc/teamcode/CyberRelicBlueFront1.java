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

@Autonomous(name = "RelicBlueFront1", group = "RiderModes")
public class CyberRelicBlueFront1 extends CyberRelicAbstract{

    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);  //Added so opMode does not sleep

    //OpenGLMatrix lastLocation = null;


    //VuforiaLocalizer vuforia;
    //BNO055IMU imu;
    @Override
    public void init() {

        super.init();

        colorSensor.enableLed(true);

    }

    //Added start to reset encoders when Play is pressed
    @Override
    public void start() {
        motorRightA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlyphLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        timer.reset();
    }

    @Override
    public void loop() {

        switch (seqRobot) {

            case 1:
                telemetry.addData("1", true);
                telemetry.update();
                motorRightA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorLeftA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorRightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorLeftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorGlyphLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("Time", System.currentTimeMillis());
                telemetry.update();
                if(chkMove(motorRightA, 0, 20)) {
                    seqRobot++;
                    timer.reset();
                }
                break;


            case 2:
                telemetry.addData("2", true);
                telemetry.addData("Time", System.currentTimeMillis());
                telemetry.update();
                servoGlyph1.setPosition(GLYPH_1_GRAB);
                servoGlyph2.setPosition(GLYPH_2_GRAB);
                servoGem.setPosition(1);
                motorGlyphLift.setTargetPosition(-7500);
                motorGlyphLift.setPower(.25);
                //slp(1000);  Setpoints are not written until the end of the loop. Don't use sleep methods in iterative opModes
                if(timer.milliseconds() > 1000) {
                    seqRobot++;
                    timer.reset();
                }
                break;


            case 3:
                telemetry.addData("3", true);
                telemetry.addData("Time", System.currentTimeMillis());
                telemetry.addData("Gyro", gyro());
                telemetry.addData("Sensor Sees: ", seeJewel(colorSensor.red(), colorSensor.blue(), hsvValues[0]));
                telemetry.addData("RedVal", colorSensor.red());
                telemetry.addData("BlueVal", colorSensor.blue());
                telemetry.update();
                colorSensor.getClass();

                if (colorSensor.red() >= colorSensor.blue()) {
                    if(rangeSensorB.cmUltrasonic() > 25){
                        motorLeftA.setPower(-.5);
                        motorLeftB.setPower(-.5);
                        motorRightA.setPower(-.5);
                        motorRightB.setPower(-.5);
                    }else{
                        motorLeftA.setPower(0);
                        motorLeftB.setPower(0);
                        motorRightA.setPower(0);
                        motorRightB.setPower(0);
                    }
                    /*
                    if(gyro() > 180 && gyro() < 300){
                        motorLeftA.setPower(0);
                        motorLeftB.setPower(0);
                        motorRightA.setPower(0);
                        motorRightB.setPower(0);
                    } else{
                        motorLeftA.setPower(-.3);
                        motorLeftB.setPower(-.3);
                        motorRightA.setPower(.3);
                        motorRightB.setPower(.3);
                    }
                    */
                }

                if (colorSensor.blue() >= colorSensor.red()) {
                    if(rangeSensorB.cmUltrasonic() > 90){
                        motorLeftA.setPower(.5);
                        motorLeftB.setPower(.5);
                        motorRightA.setPower(.5);
                        motorRightB.setPower(.5);
                    }else{
                        motorLeftA.setPower(0);
                        motorLeftB.setPower(0);
                        motorRightA.setPower(0);
                        motorRightB.setPower(0);
                    }
                    /*
                    if (gyro() < 60) {
                        motorLeftA.setPower(.3);
                        motorLeftB.setPower(.3);
                        motorRightA.setPower(-.3);
                        motorRightB.setPower(-.3);
                    }
                    */
                }
                //slp(750);
                if(timer.milliseconds() > 1750) {
                    seqRobot++;
                    timer.reset();
                }
                break;

            case 4:
                telemetry.addData("4", true);
                telemetry.addData("Time", System.currentTimeMillis());
                telemetry.update();
                servoGem.setPosition(0.1);

                //Scan VuMark


                if (rangeSensorB.cmUltrasonic() < 30){
                    motorLeftA.setPower(0);
                    motorLeftB.setPower(0);
                    motorRightA.setPower(0);
                    motorRightB.setPower(0);
                }else if (rangeSensorB.cmUltrasonic() > 30){
                    motorLeftA.setPower(-.7);
                    motorLeftB.setPower(-.7);
                    motorRightA.setPower(-.7);
                    motorRightB.setPower(-.7);
                }

                /*
                if (gyro() < 1 && gyro() > 359) {
                    motorLeftA.setPower(0);
                    motorLeftB.setPower(0);
                    motorRightA.setPower(0);
                    motorRightB.setPower(0);
                }
                else if (gyro() > 0 && gyro() <179 ){
                    motorLeftA.setPower(-.3);
                    motorLeftB.setPower(-.3);
                    motorRightA.setPower(.3);
                    motorRightB.setPower(.3);
                }
                else if(gyro() < 360 && gyro() > 181)
                {
                    motorLeftA.setPower(.3);
                    motorLeftB.setPower(.3);
                    motorRightA.setPower(-.3);
                    motorRightB.setPower(-.3);
                }
                //slp(750);
                if (gyro() < 1 && gyro() > 359) {
                    motorLeftA.setPower(0);
                    motorLeftB.setPower(0);
                    motorRightA.setPower(0);
                    motorRightB.setPower(0);
                    seqRobot++;
                }
                */
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
