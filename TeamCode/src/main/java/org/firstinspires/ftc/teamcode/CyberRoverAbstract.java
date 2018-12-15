
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public abstract class CyberRoverAbstract extends OpMode {

    BNO055IMU imu;

    // Establish Controller and Device String Constants
    // These names need to match the Robot Controller configuration file device names.
    final static String

            MOTOR_DRIVE_LEFT_A = "leftA",
            MOTOR_DRIVE_LEFT_B = "leftB",
            MOTOR_DRIVE_RIGHT_A = "rightA",
            MOTOR_DRIVE_RIGHT_B = "rightB",
            MOTOR_LIFT = "lift",
            SERVO_LOCK = "lock",
            SENSOR_COLOR = "color";

    protected boolean                  // Used to detect initial press of "A" button on gamepad 1
            pulseCaseMoveDone, // Case move complete pulse
            red,blue,
            fieldOrient,
            bDirection,
            locked;
    protected DcMotor
            motorLeftA, motorLeftB,
            motorRightA, motorRightB,
            motorLift;
    // Auto: Values used to determine current color detected
    protected float
            targetDrDistInch,                   // Targets for motor moves in sequence (engineering units)
            targetDrRotateDeg,
            hsvValues[] = {0F, 0F, 0F},
            powerLeftA, powerLeftB,
            powerRightA, powerRightB,
            powerLift,
            lock,
            velocityDrive, strafeDrive, rotationDrive;
    protected Servo
            servoLock;

    // Establish Integer Variables
    protected int
            seqRobot, target, // Switch operation integer used to identify sequence step.
            targetPosLeftA, targetPosLeftB,
            targetPosRightA, targetPosRightB;      // Drive train motor target variables (encoder counts)

    // Establish Integer Constants
    final static int
            INC_VAL = 1,
            LIFT_UP = -630, LIFT_DOWN = 0;
    // Establish Float Constants
    final static float
            SERVO_LOCKED = 0.639f,
            SERVO_UNLOCKED = 0.905f;

    // Establish Double Constants
    final static double
            DELAY_DRV_MOV_DONE = 0.1d,        // Hold/wait 0.1s after drive train move complete (seconds)
            ENCODER_CNT_PER_IN_DRIVE = 59.41979167d; // (28 count/motor rev x 40 motor rev / shaft rev) / (6" dia. wheel x pi)

    protected double
            targetPower, // General motor power variable (%, -1.0 to 1.0)
            temp,
            x, y,
            tbd;

    //------------------------------------------------------------------
    // Robot Initialization Method
    //------------------------------------------------------------------
    @Override
    public void init() {
        // Get references to dc motors and set initial mode and direction
        // It appears all encoders are reset upon robot startup, but just in case, set all motor
        // modes to Stop-And-Reset-Encoders during initialization.
        motorLeftA = hardwareMap.dcMotor.get(MOTOR_DRIVE_LEFT_A);
        motorLeftA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftA.setDirection(DcMotor.Direction.FORWARD);

        motorLeftB = hardwareMap.dcMotor.get(MOTOR_DRIVE_LEFT_B);
        motorLeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftB.setDirection(DcMotor.Direction.FORWARD);

        motorRightA = hardwareMap.dcMotor.get(MOTOR_DRIVE_RIGHT_A);
        motorRightA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightA.setDirection(DcMotor.Direction.REVERSE);

        motorRightB = hardwareMap.dcMotor.get(MOTOR_DRIVE_RIGHT_B);
        motorRightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightB.setDirection(DcMotor.Direction.REVERSE);

        motorLift = hardwareMap.dcMotor.get(MOTOR_LIFT);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setDirection(DcMotor.Direction.FORWARD);

        servoLock = hardwareMap.servo.get(SERVO_LOCK);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        seqRobot = 1;

        bDirection = true;
    } // End OpMode Initialization Method

    //------------------------------------------------------------------
    // Loop Method
    //------------------------------------------------------------------
    @Override
    public void loop()
    {
    }



    //------------------------------------------------------------------
    // Stop Method
    //------------------------------------------------------------------
    @Override
    public void stop()
    {    // stop all the motors when the program is stopped
        motorRightA.setPower(0);
        motorRightB.setPower(0);
        motorLeftA.setPower(0);
        motorLeftB.setPower(0);
        motorLift.setPower(0);
    } // End OpMode Stop Method

    double gyro() {
        return imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    //------------------------------------------------------------------
    // Miscellaneous Methods
    //------------------------------------------------------------------

    // calcRotate Method
    // Calculate linear distance needed for desired rotation
    // Parameters:
    // 		rotateDeg = Rotation desired (degrees)
    //		anglePerIn = Angle of rotation when left and right move 1 inch in opposite directions
    // Return: linear distance in inches
    float calcRotate(float rotateDeg, float anglePerIn)
    {
        return rotateDeg / anglePerIn;
    }

    // cmdMoveR Method
    // Convert desired distance from inches to encoder counts, establish new motor target, and set
    // motor power. New motor target is assumed to be relative; in other words, motor target is
    // current position plus new distance.
    // Parameters:
    //		distIn = Relative target distance (inches)
    //		encoderCntPerIn = encoder-to-inches conversion
    //		power = desired motor power (%)
    //		motor = motor
    // Return: New target (encoder counts)

    int cmdMoveR(float distIn, float encoderCntPerIn, double power, DcMotor motor)
    {
        // Solve for encoder count target. (int) needed to cast result as integer
        int target = ((int) (distIn * encoderCntPerIn));// + motor.getCurrentPosition();

        // Set motor target and power
        motor.setPower(power);
        motor.setTargetPosition(target);

        return target;
    }

    // cmdMoveA Method
    // Convert desired distance from inches to encoder counts, establish new motor target, and set
    // motor power. New motor target is assumed to be absolute; in other words, motor target is
    // based on the original home position.
    // Parameters:
    //		distIn = Absolute target distance (inches)
    //		encoderCntPerIn = encoder-to-inches conversion
    //		power = desired motor power (%)
    //		motor = motor
    // Return: New target (encoder counts)

    int cmdMoveA(float distIn, float encoderCntPerIn, double power, DcMotor motor)
    {
        // Solve for encoder count target. (int) needed to cast result as integer
        int target = (int) (distIn * encoderCntPerIn);

        // Set motor target and power
        motor.setTargetPosition(target);
        motor.setPower(power);

        return target;
    }

    // chkMove method
    // Verify motor has achieved target
    // Parameters:
    //		motor = motor
    //		target = desired target (encoder counts)
    //		delta = allowed +/- error from target (encoder counts)
    // Return:
    //		True if move complete
    //		False if move not complete

    boolean chkMove(DcMotor motor, int target, int delta)
    {
        int currentPos = motor.getCurrentPosition();
        return ((currentPos >= (target - delta)) && (currentPos <= (target + delta)));
    }

    char seeJewel(int redVal, int blueVal, float hueVal)
    {
        {
            if (redVal > blueVal)
            {
                return 'R';
            }
            else if (blueVal > redVal)
            {
                return 'B';
            }
            else
            {
                return 'N';
            }
        }
    }


    //------------------------------------------------------------------
    // Miscellaneous Methods
    //------------------------------------------------------------------

    //	scaleInput method
    // (written by unknown FTC engineer)
    // 	This method scales the joystick input so for low joystick values, the
    //	scaled value is less than linear.  This is to make it easier to drive
    //	the robot more precisely at slower speeds.

    static double scaleInput(double dVal)
    {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0)
        {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16)
        {
            index = 16;
        }

        // get value from the array.
        double dScale;
        if (dVal < 0)
        {
            dScale = -scaleArray[index];
        }
        else
        {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }


    // limit method - Recommend not using this method
    // This method prevents over-extended motor movement. Once a limit is reached, you cannot go
    // any further, but you may reverse course. Unfortunately this does not prevent significant
    // overshoot. A better way to limit motor distance is to place the motor into Run-To-Position
    // mode, and then adjust power manually.
    //
    // Method Parameters:
    //     powerValue = desired motor throttle value
    //     direction = (once the encoders are wired properly, this term may go away)
    //     lower limit / upper limit = allowed range of movement
    //
    //  Motor Output:
    //      powerValue = recalculated motor throttle
    //
    //  If time permits, you can revise the code to reduce motor power as a limit is approached.

    float limit(float powerValue, double currentPos, double lowerLimit, double upperLimit)
    {
        if (currentPos > upperLimit)
        {
            if (powerValue > 0)
            {
                powerValue = 0;
            }
        }

        if (currentPos < lowerLimit)
        {
            if (powerValue < 0)
            {
                powerValue = 0;
            }
        }

        return powerValue;
    }

}