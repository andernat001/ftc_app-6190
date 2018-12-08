package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "RoverBot")
public class CyberRoverTeleOp extends CyberRoverAbstract{
    public CyberRoverTeleOp() {
    }

    @Override
    public void init() {

        super.init();

        // Set all motors to run without encoders
        motorRightA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fieldOrient = false;
        bDirection = true;
        locked = false;

    }

    @Override
    public void loop() {

        super.loop();

        // Set drive motor power
        motorRightA.setPower(powerRightA);
        motorRightB.setPower(powerRightB);
        motorLeftA.setPower(powerLeftA);
        motorLeftB.setPower(powerLeftB);

        //Create dead-zone for drive train controls
        if (gamepad1.left_trigger <= 0.1) {
            gamepad1.left_trigger = 0;
        }

        if (gamepad1.right_trigger <= 0.1) {
            gamepad1.right_trigger = 0;
        }

        if (gamepad1.left_stick_y <= 0.05 && gamepad1.left_stick_y >= -0.05) {
            gamepad1.left_stick_y = 0;
        }

        if (gamepad1.right_stick_x <= 0.05 && gamepad1.right_stick_x >= -0.05) {
            gamepad1.right_stick_x = 0;
        }

        // Set controls for drive train
        velocityDrive = gamepad1.left_stick_y;
        if (gamepad1.left_trigger >= 0.05)
        {
            strafeDrive = - gamepad1.left_trigger;
        } else if (gamepad1.right_trigger >= 0.05)
        {
            strafeDrive = gamepad1.right_trigger;
        } else
        {
            strafeDrive = 0;
        }
        rotationDrive = -gamepad1.right_stick_x;

        //Field-Oriented on/off
        if (gamepad1.y)
        {
            fieldOrient = true;
        }
        if (gamepad1.a)
        {
            fieldOrient = false;
        }

        //Set doubles x and y
        x = strafeDrive;
        y = velocityDrive;

        //Field-Oriented drive Algorithm
        if (fieldOrient)
        {
            temp = y * Math.cos(Math.toDegrees(gyro())) + x * Math.sin(Math.toDegrees(gyro()));
            x = -y * Math.sin(Math.toDegrees(gyro())) + x * Math.cos(Math.toDegrees(gyro()));
            y = temp;
        }


        //Set floats strafeDrive and velocityDrive
        strafeDrive = (float) x;
        velocityDrive = (float) y;

        // Scale drive motor power for better control at low power
        powerRightA = (float) scaleInput(powerRightA);
        powerRightB = (float) scaleInput(powerRightB);
        powerLeftA = (float) scaleInput(powerLeftA);
        powerLeftB = (float) scaleInput(powerLeftB);




        // If the left stick and the right stick are used at the same time it halves the power of
        // the motors for better accuracy
        if (gamepad1.left_stick_y > 0.05 || gamepad1.left_stick_y < -0.05 && gamepad1.right_stick_x
                > 0.05 || gamepad1.right_stick_x < -0.05 || gamepad1.left_trigger > 0.05 ||
                gamepad1.right_trigger > 0.05) {
            powerRightA = Range.clip(powerRightA, -0.5f, 0.5f);
            powerRightB = Range.clip(powerRightB, -0.5f, 0.5f);
            powerLeftA = Range.clip(powerLeftA, -0.5f, 0.5f);
            powerLeftB = Range.clip(powerLeftB, -0.5f, 0.5f);

        } else if (gamepad1.left_trigger > 0.05 && gamepad1.right_stick_x > 0.05 ||
                gamepad1.right_stick_x < -0.05 || gamepad1.right_trigger > 0.05)
        {
            powerRightA = Range.clip(powerRightA, -0.5f, 0.5f);
            powerRightB = Range.clip(powerRightB, -0.5f, 0.5f);
            powerLeftA = Range.clip(powerLeftA, -0.5f, 0.5f);
            powerLeftB = Range.clip(powerLeftB, -0.5f, 0.5f);

        } else if (gamepad1.right_trigger > 0.05 && gamepad1.right_stick_x > 0.05 ||
                gamepad1.right_stick_x < -0.05)
        {
            powerRightA = Range.clip(powerRightA, -0.5f, 0.5f);
            powerRightB = Range.clip(powerRightB, -0.5f, 0.5f);
            powerLeftA = Range.clip(powerLeftA, -0.5f, 0.5f);
            powerLeftB = Range.clip(powerLeftB, -0.5f, 0.5f);
        } else
        {
            powerRightA = Range.clip(powerRightA, -1, 1);
            powerRightB = Range.clip(powerRightB, -1, 1);
            powerLeftA = Range.clip(powerLeftA, -1, 1);
            powerLeftB = Range.clip(powerLeftB, -1, 1);
        }

        //Switch Drive
        if (gamepad1.dpad_up)
        {
            bDirection = true; // Arm is front.
            slp(500);
        }
        if (gamepad1.dpad_down)
        {
            bDirection = false; // Collection is front
            slp(500);
        }

        // Sets bDirection to true when field-oriented driving is on, so field-orient controls are
        // not flipped
        if (fieldOrient)
        {
            bDirection = true;
        }

        // Flips direction controls for backwards navigation
        if (bDirection) // Front is front
        {
            powerRightA = velocityDrive - rotationDrive + strafeDrive;
            powerRightB = velocityDrive - rotationDrive - strafeDrive;
            powerLeftA = velocityDrive + rotationDrive - strafeDrive;
            powerLeftB = velocityDrive + rotationDrive + strafeDrive;
        } else  // Back is front
        {
            powerRightA = -velocityDrive - rotationDrive - strafeDrive;
            powerRightB = -velocityDrive - rotationDrive + strafeDrive;
            powerLeftA = -velocityDrive + rotationDrive + strafeDrive;
            powerLeftB = -velocityDrive + rotationDrive - strafeDrive;
        }

        // Lift program controls

        /*if (gamepad2.y)
        {
            motorLift.setTargetPosition(LIFT_UP);
        }
        if (gamepad2.a)
        {
            motorLift.setTargetPosition(LIFT_DOWN);
        }*/
        powerLift = gamepad2.left_stick_y; // Divide power by two, so the motor slows down
        powerLift = (float) scaleInput(powerLift); // Scale the power of the motor to how far the
                                                   // joystick is pressed
        motorLift.setPower(powerLift);


        if(gamepad2.b && locked) // Press b to lock and press it again to unlock it
        {
            locked = false;
            slp(75); // The program will wait, so it does not think the button is pressed
                            // again before you release it
        }
        if(!locked)
        {
            servoLock.setPosition(SERVO_UNLOCKED);
        }

        if(gamepad2.b && !locked)
        {
            locked = true;
            slp(75);// The program will wait, so it does not think the button is pressed
                           // again before you release it
        }
        if(locked)
        {
            servoLock.setPosition(SERVO_LOCKED);
        }

        /* // Controls used to test the needed position of the servo
        if (gamepad2.y)
        {
            lock = lock + INC_VAL;
            servoLock.setPosition(lock/180);
            slp(100);

        }

        if (gamepad2.a)
        {
            if (lock > 0)
            {
                lock = lock - INC_VAL;
            }
            else
            {
                lock = 0;
            }
            servoLock.setPosition(lock/180);
            slp(100);

        }*/






        telemetry.addData("Locked: ", locked);
        telemetry.addData("Lock Position:", servoLock.getPosition());
        telemetry.addData("Lift Position", motorLift.getCurrentPosition());
        telemetry.update();

// End OpMode Loop Method
    }
    private void slp(int slptime) {
        try {
            Thread.sleep(slptime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    @Override
    public void stop ()
    {
        super.stop();
    }
}