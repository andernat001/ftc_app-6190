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

/**
 * Created by Steve on 11/11/2017.
 */
@Autonomous(name = "RelicBlueBack", group = "RiderModes")
public class CyberRelicBlueBack extends CyberRelicAbstract {
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);  //Added so opMode does not sleep
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    BNO055IMU imu;
    @Override
    public void init() {
        super.init();
        colorSensor.enableLed(true);

    }
    @Override
    public void loop(){
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
                if (chkMove(motorRightA, 0, 20)) {
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
                servoGem.setPosition(.9);
                motorGlyphLift.setTargetPosition(-3500);
                motorGlyphLift.setPower(.5);
                //slp(1000);  Setpoints are not written until the end of the loop. Don't use sleep methods in iterative opModes
                if (timer.milliseconds() > 1000) {
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
                    if (rangeSensorB.cmUltrasonic() > 115) {
                        motorLeftA.setPower(-.5);
                        motorLeftB.setPower(-.5);
                        motorRightA.setPower(-.5);
                        motorRightB.setPower(-.5);
                    } else {
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
                    if (rangeSensorB.cmUltrasonic() > 90) {
                        motorLeftA.setPower(.5);
                        motorLeftB.setPower(.5);
                        motorRightA.setPower(.5);
                        motorRightB.setPower(.5);
                    } else {
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
                if (timer.milliseconds() > 1750) {
                    seqRobot++;
                    timer.reset();
                }
                break;

            case 4:
                servoGem.setPosition(.1);
                /*
                if (gyro() < 1 && gyro() > 359) {
                    motorLeftA.setPower(0);
                    motorLeftB.setPower(0);
                    motorRightA.setPower(0);
                    motorRightB.setPower(0);
                }
                else if (gyro() > 0 && gyro() <179 ){
                    motorLeftA.setPower(-.1);
                    motorLeftB.setPower(-.1);
                    motorRightA.setPower(.1);
                    motorRightB.setPower(.1);
                }
                else if(gyro() < 360 && gyro() > 181)
                {
                    motorLeftA.setPower(.1);
                    motorLeftB.setPower(.1);
                    motorRightA.setPower(-.1);
                    motorRightB.setPower(-.1);
                }*/
                if (timer.milliseconds() > 750) {
                    seqRobot+=6;
                    timer.reset();
                }
                break;


            case 23:
            case 21:
            case 18:
            case 15:
            case 13:
            case 11:
            //case 9:
            //case 5:
                motorRightA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftA.setTargetPosition(0);
                motorLeftB.setTargetPosition(0);
                motorRightA.setTargetPosition(0);
                motorRightB.setTargetPosition(0);
                seqRobot++;
                break;
            /*
            case 6: {
                if(gyro() > 180 && gyro() <= 250){
                    motorLeftA.setPower(0);
                    motorLeftB.setPower(0);
                    motorRightA.setPower(0);
                    motorRightB.setPower(0);
                }else{
                    motorLeftA.setPower(-.1);
                    motorLeftB.setPower(-.1);
                    motorRightA.setPower(.1);
                    motorRightB.setPower(.1);
                }
                seqRobot++;
                break;
            }

            case 7: {
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
                parameters.vuforiaLicenseKey = "AdlfowT/////AAAAGUpRoaEvyUfNtuTDeKMo6qEf2Y8oPuvPan17xUGgdDWoYKTx+JNrzPv2tPPmKMQcyOw9MNnOeGDXCPFCDOOKjsUTjil2cGK9odRVmSWL0xsxdsxtbz9Y3ZW2q1fi9IJsBofvfxfTa/6t9JDldr1+6lcBi9izU2k0ZC9Md6S8DHkcvQ7Q7P9NRepmQZXU+ztVWxB9gNHJ1128u3zADXS+pIkW9qIUHfc6UybysSNpmeh65VxdFRu2Tnlwh3fqAB+NjG9eMmgP49FyW3C3wnkwMMCVqT4JBdhRPviRHyp7lXXtzVqr/BB30ww0hk0W7gyiANIbVvQq/04i18SFFuS5H9zX2v0g5J3ViTvfUybve/AA";
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
                VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
                VuforiaTrackable relicTemplate = relicTrackables.get(0);
                relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
                relicTrackables.activate();
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN){
                    telemetry.addData("VuMark", "%s visible", vuMark);
                    if (vuMark == RelicRecoveryVuMark.LEFT){
                        leftCol = true;
                        centerCol = false;
                        rightCol = false;
                    }
                    else if (vuMark == RelicRecoveryVuMark.CENTER){
                        leftCol = false;
                        centerCol = true;
                        rightCol = false;
                    }
                    else if (vuMark == RelicRecoveryVuMark.RIGHT){
                        leftCol = false;
                        centerCol = false;
                        rightCol = true;
                    }
                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                    if (pose != null){
                        VectorF trans = pose.getTranslation();
                        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                        // Extract the X, Y, and Z components of the offset of the target relative to the robot
                        double tX = trans.get(0);
                        double tY = trans.get(1);
                        double tZ = trans.get(2);
                        // Extract the rotational components of the target relative to the robot
                        double rX = rot.firstAngle;
                        double rY = rot.secondAngle;
                        double rZ = rot.thirdAngle;
                    }
                }
                else{
                    telemetry.addData("VuMark", "not visible");
                }
                telemetry.update();
                seqRobot++;
                break;
            }

            case 8: {
                if (gyro() < 1 || gyro() > 359) {
                    motorLeftA.setPower(0);
                    motorLeftB.setPower(0);
                    motorRightA.setPower(0);
                    motorRightB.setPower(0);
                }
                else if (gyro() > 1 && gyro() < 179 ){
                    motorLeftA.setPower(-.1);
                    motorLeftB.setPower(-.1);
                    motorRightA.setPower(.1);
                    motorRightB.setPower(.1);
                }
                else if(gyro() < 359 && gyro() > 181)
                {
                    motorLeftA.setPower(.1);
                    motorLeftB.setPower(.1);
                    motorRightA.setPower(-.1);
                    motorRightB.setPower(-.1);
                }
                seqRobot++;
                break;
            }*/
            case 10:
                if (rangeSensorF.cmUltrasonic() > 24){
                    motorLeftA.setPower(.1);
                    motorLeftB.setPower(.1);
                    motorRightA.setPower(.1);
                    motorRightB.setPower(.1);
                } else{
                    motorLeftA.setPower(0);
                    motorLeftB.setPower(0);
                    motorRightA.setPower(0);
                    motorRightB.setPower(0);
                }
                if (timer.milliseconds() > 750) {
                    seqRobot++;
                    timer.reset();
                }
                break;

            case 12:
                if (gyro() < 91 && gyro() > 89) {
                    motorLeftA.setPower(0);
                    motorLeftB.setPower(0);
                    motorRightA.setPower(0);
                    motorRightB.setPower(0);
                }
                else if (gyro() > 91){
                    motorLeftA.setPower(-.1);
                    motorLeftB.setPower(-.1);
                    motorRightA.setPower(.1);
                    motorRightB.setPower(.1);
                }
                else if(gyro() < 89)
                {
                    motorLeftA.setPower(.1);
                    motorLeftB.setPower(.1);
                    motorRightA.setPower(-.1);
                    motorRightB.setPower(-.1);
                }
                if (timer.milliseconds() > 500) {
                    seqRobot++;
                    timer.reset();
                }
                break;

            case 14:
                if (leftCol){
                    if (rangeSensorB.cmUltrasonic() < 48){
                        motorLeftA.setPower(.1);
                        motorLeftB.setPower(.1);
                        motorRightA.setPower(.1);
                        motorRightB.setPower(.1);
                    }if (rangeSensorB.cmUltrasonic() >= 49) {
                        motorLeftA.setPower(-.05);
                        motorLeftB.setPower(-.05);
                        motorRightA.setPower(-.05);
                        motorRightB.setPower(-.05);
                    }
                }
                if (centerCol){
                    if (rangeSensorB.cmUltrasonic() < 68){
                        motorLeftA.setPower(.1);
                        motorLeftB.setPower(.1);
                        motorRightA.setPower(.1);
                        motorRightB.setPower(.1);
                    }if (rangeSensorB.cmUltrasonic() >= 69) {
                        motorLeftA.setPower(-.05);
                        motorLeftB.setPower(-.05);
                        motorRightA.setPower(-.05);
                        motorRightB.setPower(-.05);
                    }
                }
                if (rightCol){
                    if (rangeSensorB.cmUltrasonic() < 86){
                        motorLeftA.setPower(.1);
                        motorLeftB.setPower(.1);
                        motorRightA.setPower(.1);
                        motorRightB.setPower(.1);
                    }if (rangeSensorB.cmUltrasonic() >= 87) {
                        motorLeftA.setPower(-.05);
                        motorLeftB.setPower(-.05);
                        motorRightA.setPower(-.05);
                        motorRightB.setPower(-.05);
                    }
                }
                if (timer.milliseconds() > 1250) {
                    seqRobot++;
                    timer.reset();
                }
                break;

            case 16:
                if (gyro() < 1 || gyro() > 359) {
                    motorLeftA.setPower(0);
                    motorLeftB.setPower(0);
                    motorRightA.setPower(0);
                    motorRightB.setPower(0);
                }
                else if (gyro() > 1 && gyro() < 179 ){
                    motorLeftA.setPower(-.1);
                    motorLeftB.setPower(-.1);
                    motorRightA.setPower(.1);
                    motorRightB.setPower(.1);
                }
                else if(gyro() < 359 && gyro() > 181)
                {
                    motorLeftA.setPower(.1);
                    motorLeftB.setPower(.1);
                    motorRightA.setPower(-.1);
                    motorRightB.setPower(-.1);
                }
                if (timer.milliseconds() > 500) {
                    seqRobot++;
                    timer.reset();
                }
                break;

            case 17:
                if (rangeSensorF.cmUltrasonic() > 22){
                    motorLeftA.setPower(.1);
                    motorLeftB.setPower(.1);
                    motorRightA.setPower(.1);
                    motorRightB.setPower(.1);
                }
                if (timer.milliseconds() > 1000) {
                    seqRobot++;
                    timer.reset();
                }
                break;

            case 19:
                servoGlyph1.setPosition(GLYPH_1_RELEASE);
                servoGlyph2.setPosition(GLYPH_2_RELEASE);
                if (timer.milliseconds() > 500) {
                    seqRobot++;
                    timer.reset();
                }
                break;

            case 20:
                if (rangeSensorF.cmUltrasonic() < 5){
                    motorLeftA.setPower(-.2);
                    motorRightA.setPower(-.2);
                    motorLeftB.setPower(-.2);
                    motorRightB.setPower(-.2);
                } else {
                    motorLeftA.setPower(0);
                    motorRightA.setPower(0);
                    motorLeftB.setPower(0);
                    motorRightB.setPower(0);
                }
                if (timer.milliseconds() > 750) {
                    seqRobot++;
                    timer.reset();
                }
                    break;

            case 22:
                if (gyro() < 90.25 || gyro() > 89.75) {
                    motorLeftA.setPower(0);
                    motorLeftB.setPower(0);
                    motorRightA.setPower(0);
                    motorRightB.setPower(0);
                }else{
                    motorLeftA.setPower(.1);
                    motorLeftB.setPower(.1);
                    motorRightA.setPower(-.1);
                    motorRightB.setPower(-.1);
                }
                if (timer.milliseconds() > 1000) {
                    seqRobot++;
                    timer.reset();
                }
                break;

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
    private void slp(int slptime) {
        try {
            sleep(slptime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    }