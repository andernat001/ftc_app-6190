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

@Autonomous(name = "RelicBlueFront2", group = "RiderModes")
public class CyberRelicBlueFront2 extends CyberRelicAbstract{

    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);  //Added so opMode does not sleep

    //OpenGLMatrix lastLocation = null;


    VuforiaLocalizer vuforia;
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
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                    telemetry.addData("VuMark", "%s visible", vuMark);

                    if (vuMark == RelicRecoveryVuMark.LEFT) {
                        leftCol = true;
                        centerCol = false;
                        rightCol = false;
                    } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                        leftCol = false;
                        centerCol = true;
                        rightCol = false;
                    } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                        leftCol = false;
                        centerCol = false;
                        rightCol = true;
                    } else if (vuMark == RelicRecoveryVuMark.UNKNOWN){
                        leftCol = true;
                        centerCol = false;
                        leftCol = false;
                    }

                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();

                    if (pose != null) {
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
                } else {
                    telemetry.addData("VuMark", "not visible");
                }
                telemetry.addData("2", true);
                telemetry.update();
                if(timer.milliseconds() > 1250) {
                    seqRobot++;
                    timer.reset();
                }
                break;


            case 3:
                telemetry.addData("3", true);
                telemetry.update();
                if (leftCol) {
                    if (rangeSensorB.cmUltrasonic() < 110) {
                        motorLeftA.setPower(.5);
                        motorLeftB.setPower(.5);
                        motorRightA.setPower(.5);
                        motorRightB.setPower(.5);
                    }
                    if (rangeSensorB.cmUltrasonic() > 111) {
                        motorLeftA.setPower(-.05);
                        motorLeftB.setPower(-.05);
                        motorRightA.setPower(-.05);
                        motorRightB.setPower(-.05);
                    }
                }

                if (centerCol) {
                    if (rangeSensorB.cmUltrasonic() < 127.5) {
                        motorLeftA.setPower(.5);
                        motorLeftB.setPower(.5);
                        motorRightA.setPower(.5);
                        motorRightB.setPower(.5);
                    }
                    if (rangeSensorB.cmUltrasonic() >= 128.5) {
                        motorLeftA.setPower(-.05);
                        motorLeftB.setPower(-.05);
                        motorRightA.setPower(-.05);
                        motorRightB.setPower(-.05);
                    }
                }

                if (rightCol) {
                    if (rangeSensorB.cmUltrasonic() < 143) {
                        motorLeftA.setPower(.5);
                        motorLeftB.setPower(.5);
                        motorRightA.setPower(.5);
                        motorRightB.setPower(.5);
                    } else if (rangeSensorB.cmUltrasonic() > 144) {
                        motorLeftA.setPower(-.05);
                        motorLeftB.setPower(-.05);
                        motorRightA.setPower(-.05);
                        motorRightB.setPower(-.05);
                    }
                }
                if(timer.milliseconds() > 1000) {
                    seqRobot++;
                    timer.reset();
                }
                break;

            case 4:
                telemetry.addData("4", true);
                telemetry.update();
                if(gyro() < 1 || gyro() > 359){
                    motorLeftA.setPower(0);
                    motorLeftB.setPower(0);
                    motorRightA.setPower(0);
                    motorRightB.setPower(0);
                }
                else if (gyro() > 0 && gyro() < 179) {
                    motorLeftA.setPower(-.1);
                    motorLeftB.setPower(-.1);
                    motorRightA.setPower(.1);
                    motorRightB.setPower(.1);
                } else if (gyro() < 360 && gyro() > 181) {
                    motorLeftA.setPower(.1);
                    motorLeftB.setPower(.1);
                    motorRightA.setPower(-.1);
                    motorRightB.setPower(-.1);
                }
                if(timer.milliseconds() > 1000) {
                    seqRobot++;
                    timer.reset();
                }
                break;

            case 5:
                telemetry.addData("5", true);
                telemetry.update();

                if (leftCol) {

                    if (rangeSensorB.cmUltrasonic() < 110) {
                        motorLeftA.setPower(.05);
                        motorLeftB.setPower(.05);
                        motorRightA.setPower(.05);
                        motorRightB.setPower(.05);
                    }else if (rangeSensorB.cmUltrasonic() > 115){
                        motorLeftA.setPower(-.05);
                        motorLeftB.setPower(-.05);
                        motorRightA.setPower(-.05);
                        motorRightB.setPower(-.05);
                    }

                }

                if (centerCol) {
                    if (rangeSensorB.cmUltrasonic() < 127) {
                        motorLeftA.setPower(.05);
                        motorLeftB.setPower(.05);
                        motorRightA.setPower(.05);
                        motorRightB.setPower(.05);
                    }else if (rangeSensorB.cmUltrasonic() > 132){
                        motorLeftA.setPower(-.05);
                        motorLeftB.setPower(-.05);
                        motorRightA.setPower(-.05);
                        motorRightB.setPower(-.05);
                    }

                }

                if (rightCol) {
                    if (rangeSensorB.cmUltrasonic() < 143) {
                        motorLeftA.setPower(.05);
                        motorLeftB.setPower(.05);
                        motorRightA.setPower(.05);
                        motorRightB.setPower(.05);
                    }else if (rangeSensorB.cmUltrasonic() > 148){
                        motorLeftA.setPower(-.05);
                        motorLeftB.setPower(-.05);
                        motorRightA.setPower(-.05);
                        motorRightB.setPower(-.05);
                    }

                }
                if(timer.milliseconds() > 1000) {
                    seqRobot++;
                    timer.reset();
                }
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
