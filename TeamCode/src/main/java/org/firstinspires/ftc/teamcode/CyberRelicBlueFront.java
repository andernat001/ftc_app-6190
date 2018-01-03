package org.firstinspires.ftc.teamcode;

    import com.qualcomm.hardware.bosch.BNO055IMU;
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.hardware.GyroSensor;

    import org.firstinspires.ftc.robotcore.external.ClassFactory;
    import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
    import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
    import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
    import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
    import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
    import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
    import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
    import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
    import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
    import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
    import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

    import static java.lang.Thread.sleep;

@Autonomous(name = "RelicBlueFront", group = "RiderModes")
public class CyberRelicBlueFront extends CyberRelicAbstract{

    OpenGLMatrix lastLocation = null;


    VuforiaLocalizer vuforia;
    BNO055IMU imu;
    @Override
    public void init() {

        super.init();

        colorSensor.enableLed(true);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    @Override
    public void loop() {

        super.loop();

        switch (seqRobot) {

            case 1: {
                telemetry.addData("1", true);
                telemetry.update();
                motorLeftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
            }

            case 2: {
                telemetry.addData("2", true);
                telemetry.update();
                servoGlyph1.setPosition(GLYPH_1_GRAB);
                servoGlyph2.setPosition(GLYPH_2_GRAB);
                gemDown();
                seqRobot++;
                break;
            }

            case 3: {
                telemetry.addData("3", true);
                telemetry.update();
                colorSensor.getClass();

                if (red) {
                    if(gyro > 180 && gyro < 350){
                        motorLeftA.setPower(0);
                        motorLeftB.setPower(0);
                        motorRightA.setPower(0);
                        motorRightB.setPower(0);
                    } else{
                        motorLeftA.setPower(-.1);
                        motorLeftB.setPower(-.1);
                        motorRightA.setPower(.1);
                        motorRightB.setPower(.1);
                    }
                }

                if (blue) {
                    if (gyro < 10) {
                        motorLeftA.setPower(.1);
                        motorLeftB.setPower(.1);
                        motorRightA.setPower(-.1);
                        motorRightB.setPower(-.1);
                    }
                }

                seqRobot++;
                break;
            }

            case 4: {
                telemetry.addData("4", true);
                telemetry.update();
                gemUp();

                if (gyro < 1 && gyro > 359) {
                    motorLeftA.setPower(0);
                    motorLeftB.setPower(0);
                    motorRightA.setPower(0);
                    motorRightB.setPower(0);
                }
                else if (gyro > 0 && gyro <179 ){
                    motorLeftA.setPower(-.1);
                    motorLeftB.setPower(-.1);
                    motorRightA.setPower(.1);
                    motorRightB.setPower(.1);
                }
                else if(gyro < 360 && gyro > 181)
                {
                    motorLeftA.setPower(.1);
                    motorLeftB.setPower(.1);
                    motorRightA.setPower(-.1);
                    motorRightB.setPower(-.1);
                }

                seqRobot++;
                break;
            }

            case 24:
            case 22:
            case 19:
            case 17:
            case 15:
            case 13:
            case 11:
            case 9:
            case 5: {
                if (seqRobot == 5){
                    telemetry.addData("5", true);
                    telemetry.update();
                }
                if (seqRobot == 9){
                    telemetry.addData("9", true);
                    telemetry.update();
                }
                if (seqRobot == 11){
                    telemetry.addData("11", true);
                    telemetry.update();
                }
                if (seqRobot == 13){
                    telemetry.addData("13", true);
                    telemetry.update();
                }
                if (seqRobot == 15){
                    telemetry.addData("15", true);
                    telemetry.update();
                }
                if (seqRobot == 17){
                    telemetry.addData("17", true);
                    telemetry.update();
                }if (seqRobot == 19){
                    telemetry.addData("19", true);
                    telemetry.update();
                }
                if (seqRobot == 22){
                    telemetry.addData("22", true);
                    telemetry.update();
                }
                if (seqRobot == 24){
                    telemetry.addData("24", true);
                    telemetry.update();
                }
                motorLeftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
            }
            case 6: {
                telemetry.addData("6", true);
                telemetry.update();
                if(gyro > 180 && gyro <= 250){
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
                telemetry.addData("7", true);
                telemetry.update();
                seqRobot++;
                break;
            }

            case 8: {
                telemetry.addData("8", true);
                telemetry.update();
                if (gyro < 1 || gyro > 359) {
                    motorLeftA.setPower(0);
                    motorLeftB.setPower(0);
                    motorRightA.setPower(0);
                    motorRightB.setPower(0);
                }
                else if (gyro > 1 && gyro < 179 ){
                    motorLeftA.setPower(-.1);
                    motorLeftB.setPower(-.1);
                    motorRightA.setPower(.1);
                    motorRightB.setPower(.1);
                }
                else if(gyro < 359 && gyro > 181)
                {
                    motorLeftA.setPower(.1);
                    motorLeftB.setPower(.1);
                    motorRightA.setPower(-.1);
                    motorRightB.setPower(-.1);
                }
                seqRobot++;
                break;
            }

            case 10: {
                telemetry.addData("10", true);
                telemetry.update();
                if (leftCol) {
                    if (rangeSensorB.cmUltrasonic() < 110) {
                        motorLeftA.setPower(.25);
                        motorLeftB.setPower(.25);
                        motorRightA.setPower(.25);
                        motorRightB.setPower(.25);
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
                        motorLeftA.setPower(.25);
                        motorLeftB.setPower(.25);
                        motorRightA.setPower(.25);
                        motorRightB.setPower(.25);
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
                        motorLeftA.setPower(.25);
                        motorLeftB.setPower(.25);
                        motorRightA.setPower(.25);
                        motorRightB.setPower(.25);
                    } else if (rangeSensorB.cmUltrasonic() > 144) {
                        motorLeftA.setPower(-.05);
                        motorLeftB.setPower(-.05);
                        motorRightA.setPower(-.05);
                        motorRightB.setPower(-.05);
                    }
                }
                seqRobot++;
                break;
            }

            case 12: {
                telemetry.addData("12", true);
                telemetry.update();
                if(gyro < 1 || gyro > 359){
                    motorLeftA.setPower(0);
                    motorLeftB.setPower(0);
                    motorRightA.setPower(0);
                    motorRightB.setPower(0);
                }
                else if (gyro > 0 && gyro < 179) {
                    motorLeftA.setPower(-.1);
                    motorLeftB.setPower(-.1);
                    motorRightA.setPower(.1);
                    motorRightB.setPower(.1);
                } else if (gyro < 360 && gyro > 181) {
                    motorLeftA.setPower(.1);
                    motorLeftB.setPower(.1);
                    motorRightA.setPower(-.1);
                    motorRightB.setPower(-.1);
                }
                seqRobot++;
                break;
            }

            case 14: {
                telemetry.addData("14", true);
                telemetry.update();

                if (leftCol) {

                    if (rangeSensorB.cmUltrasonic() < 110) {
                        motorLeftA.setPower(.05);
                        motorLeftB.setPower(.05);
                        motorRightA.setPower(.05);
                        motorRightB.setPower(.05);
                    } else {
                        seqRobot++;
                        break;
                    }
                }

                if (centerCol) {
                    if (rangeSensorB.cmUltrasonic() < 127) {
                        motorLeftA.setPower(.05);
                        motorLeftB.setPower(.05);
                        motorRightA.setPower(.05);
                        motorRightB.setPower(.05);
                    } else {
                        seqRobot++;
                        break;
                    }
                }

                if (rightCol) {
                    if (rangeSensorB.cmUltrasonic() < 143) {
                        motorLeftA.setPower(.05);
                        motorLeftB.setPower(.05);
                        motorRightA.setPower(.05);
                        motorRightB.setPower(.05);
                    } else {
                        seqRobot++;
                        break;
                    }
                }
            }

            case 16:
            {
                telemetry.addData("16", true);
                telemetry.update();
                if (gyro < 271) {
                    motorLeftA.setPower(-.1);
                    motorLeftB.setPower(-.1);
                    motorRightA.setPower(.1);
                    motorRightB.setPower(.1);
                }
                if (gyro > 269) {
                    motorLeftA.setPower(.1);
                    motorLeftB.setPower(.1);
                    motorRightA.setPower(-.1);
                    motorRightB.setPower(-.1);
                }
                seqRobot++;
                break;
            }

            case 18:
                telemetry.addData("18", true);
                telemetry.update();
            {
                if (rangeSensorF.cmUltrasonic() > 22) {
                    motorLeftA.setPower(.1);
                    motorLeftB.setPower(.1);
                    motorRightA.setPower(.1);
                    motorRightB.setPower(.1);
                } else {
                    seqRobot++;
                    break;
                }
            }

            case 20:
            {
                telemetry.addData("20", true);
                telemetry.update();
                servoGlyph1.setPosition(GLYPH_1_RELEASE);
                servoGlyph2.setPosition(GLYPH_2_RELEASE);
                seqRobot++;
                break;
            }

            case 21:
            {
                telemetry.addData("21", true);
                telemetry.update();
                if (rangeSensorF.cmUltrasonic() < 5){
                    motorLeftA.setPower(-.2);
                    motorRightA.setPower(-.2);
                    motorLeftB.setPower(-.2);
                    motorRightB.setPower(-.2);
                }
                else
                {
                    motorLeftA.setPower(0);
                    motorRightA.setPower(0);
                    motorLeftB.setPower(0);
                    motorRightB.setPower(0);
                    seqRobot++;
                    break;
                }
            }

            case 23:
            {
                telemetry.addData("23", true);
                telemetry.update();
                if (gyro < 90.25 || gyro > 89.75) {
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