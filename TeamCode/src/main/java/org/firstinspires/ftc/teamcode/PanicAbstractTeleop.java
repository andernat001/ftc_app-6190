package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;


public abstract class PanicAbstractTeleop extends OpMode {

    // motors and servos
    public static final String LEFT_MOTOR = "Left Drive Train";
    public static final String RIGHT_MOTOR = "Right Drive Train";
    protected DcMotor motorRight;
    protected DcMotor motorLeft;

    public static final String COLLECTION_SYSTEM_MOTOR = "Collection System Motor";
    protected DcMotor collectionMotor;

    public static final String BUTTON_PUSHER = "Beacon Button Pusher";
    public static final String FLYWHEEL_LAUNCHER = "Flywheel Servo Launcher";
    protected Servo buttonPusher;
    protected Servo particleFlipper;

    public static final String FRONT_FLYWHEEL_MOTOR = "Front Flywheel Motor";
    public static final String BACK_FLYWHEEL_MOTOR = "Back Flywheel Motor";
    public static final String SHOOTER = "Shooter";

    protected DcMotor frontFlywheelMotor;
    protected DcMotor backFlywheelMotor;
    protected VoltageSensor voltageSensor;

    // sensors
    public static final String BOTTOM_SENSOR = "Bottom Sensor";
    public static final String BEACON_SENSOR = "Beacon Sensor";
    public static final String BEACON_SENSOR_2 = "Beacon Sensor 2";
    public static final String RANGE_SENSOR = "Range Sensor";
    public static final String GYRO_SENSOR = "Gyro Sensor";

    protected OpticalDistanceSensor bottomSensor;
    protected ColorSensor beaconSensor;
    protected ColorSensor beaconSensor2;

    protected ModernRoboticsI2cRangeSensor rangeSensor;
    protected ModernRoboticsI2cGyro gyroSensor;

    // constant bank
    public static final int ENCODER_COUNTS = 28;
    public static final int MOTOR_COUNTS_40 = 40;

    public static final double PI = Math.PI;
    public static final double WHEEL_DIAMETER = 3.978; // :P
    public static final double SPIN_DIAMETER = 15.00;
    public static final double SPIN_CIRCUMFERENCE = SPIN_DIAMETER * PI; // 45.553093477052001957708329057553
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI;
    public static final double ENCODER_COUNTS_PER_IN = (ENCODER_COUNTS * MOTOR_COUNTS_40) / (WHEEL_CIRCUMFERENCE);
    public static final double ENCODER_COUNTS_PER_DEGREE_SPIN = ENCODER_COUNTS_PER_IN * (SPIN_CIRCUMFERENCE / 360);

    // flywheel
    protected boolean lastDpadUp = false;
    protected boolean lastDpadDown = false;
    protected double flywheelSpeed = 0.50;
    protected boolean flywheelRunning = false;

    // button pusher
    protected boolean lastLeftBumper = false;
    protected boolean lastRightBumper = false;

    // CRServo Things
    @Deprecated
    protected boolean lastDpadLeft = false;
    @Deprecated
    protected boolean lastDpadRight = false;
    protected int outTimeValue = 300;

    @Deprecated
    protected void changeButtonSpeed(int value) {
        outTimeValue = outTimeValue + value;
    }

    protected void rightButton()
    {
        long outTime = System.currentTimeMillis() + outTimeValue;
        while (System.currentTimeMillis() < outTime)
        {
            buttonPusher.setDirection(Servo.Direction.REVERSE);
            buttonPusher.setPosition(0.0);
        }
        buttonPusher.setPosition(0.5);
    }

    protected void leftButton()
    {
        long outTime = System.currentTimeMillis() + outTimeValue;
        while (System.currentTimeMillis() < outTime)
        {
            buttonPusher.setDirection(Servo.Direction.FORWARD);
            buttonPusher.setPosition(0.0);
        }
        buttonPusher.setPosition(0.5);
    }

    int inchesToEncoderCount(double inches) {
        return ((int) (inches * ENCODER_COUNTS_PER_IN));
    }

    int degreesToEncoderCount(double degrees) {
        return ((int) (degrees * ENCODER_COUNTS_PER_DEGREE_SPIN));
    }

    // Drive train things
    protected void setDriveTrainPower(double power) {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }
    protected void stopDriveTrain() {
        setDriveTrainPower(0);
    }

    // flywheel stuff
    protected void stopFlywheel() {
        setFlywheelSpeed(0);
        flywheelRunning = true;
    }

    protected void setFlywheelSpeed(double power) {
        backFlywheelMotor.setPower(power);
        frontFlywheelMotor.setPower(power);
    }

    protected void startFlywheel() {
        setFlywheelSpeed(flywheelSpeed);
        flywheelRunning = true;
    }

    // drive train reset
    protected void resetDriveTrainEncoders() {
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void buttonPusherIsForward() {
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void shooterIsForward() {
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /*
    * Code to run when the op mode is first enabled goes here
    *
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
    */
    @Override
    public void init() {
        motorRight = hardwareMap.dcMotor.get(RIGHT_MOTOR);
        motorLeft = hardwareMap.dcMotor.get(LEFT_MOTOR);
        //shooter is forward with this setting
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        collectionMotor = hardwareMap.dcMotor.get(COLLECTION_SYSTEM_MOTOR);

        buttonPusher = hardwareMap.servo.get(BUTTON_PUSHER);
        buttonPusher.setPosition(0.5);

        particleFlipper = hardwareMap.servo.get(FLYWHEEL_LAUNCHER);
        particleFlipper.setPosition(0.46);

        frontFlywheelMotor = hardwareMap.dcMotor.get(FRONT_FLYWHEEL_MOTOR);
        backFlywheelMotor = hardwareMap.dcMotor.get(BACK_FLYWHEEL_MOTOR);
        frontFlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        voltageSensor = hardwareMap.voltageSensor.get(SHOOTER);

        bottomSensor = hardwareMap.opticalDistanceSensor.get(BOTTOM_SENSOR);
        //bottomSensor.setI2cAddress(I2cAddr.create8bit(0x3C));

        beaconSensor = hardwareMap.colorSensor.get(BEACON_SENSOR);
        beaconSensor.setI2cAddress(I2cAddr.create8bit(0x4C));
        beaconSensor.enableLed(false);

        beaconSensor2 = hardwareMap.colorSensor.get(BEACON_SENSOR_2);
        beaconSensor2.setI2cAddress(I2cAddr.create7bit(0x1e));
        beaconSensor2.enableLed(false);

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, RANGE_SENSOR);

        gyroSensor = hardwareMap.get(ModernRoboticsI2cGyro.class, GYRO_SENSOR);
        gyroSensor.calibrate();

        bottomSensor.enableLed(false);
        bottomSensor.enableLed(true);
        beaconSensor.enableLed(false);
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}
