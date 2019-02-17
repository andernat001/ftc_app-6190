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
        if (gamepad1.left_trigger >= 0.05) // Left trigger strafes to the left
        {
            strafeDrive = - gamepad1.left_trigger;
        } else if (gamepad1.right_trigger >= 0.05) // Right trigger strafes to the right
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

        //Field-Oriented Drive Algorithm
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

        // If the left stick, the right stick, left trigger, and/or right trigger are used at the
        // same time it halves the power of the motors for better accuracy
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
            bDirection = true; // Front is front.
            slp(500);
        }
        if (gamepad1.dpad_down)
        {
            bDirection = false; // Back is front
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

        /*if (gamepad2.y) // Tested to see if motorLift would work in set to position mode...
        // it didn't work
        {
            motorLift.setTargetPosition(LIFT_UP);
        }
        if (gamepad2.a)
        {
            motorLift.setTargetPosition(LIFT_DOWN);
        }*/

        if (gamepad2.left_stick_y <= 0.075 || gamepad2.left_stick_y >= -0.75) // Make motorLift's
        // maximum power 0.075

        {
            powerLift = -gamepad2.left_stick_y;
            powerLift = (float) scaleInput(powerLift); // Scale the power of the motor to how far the
            // joystick is pressed
        } else {
            powerLift = 0.075f;
        }
        motorLift.setPower(powerLift);


        if(gamepad2.a && locked) // Press a to lock
        {
            locked = false;
            slp(250); // The program will wait, so it does not think the button is pressed
            // again before you release it
        }
        if(!locked)
        {
            servoLock.setPosition(SERVO_UNLOCKED);
        }

        if(gamepad2.a && !locked) // Press a again to unlock
        {
            locked = true;
            slp(250);// The program will wait, so it does not think the button is pressed
            // again before you release it

        }
        if(locked)
        {
            servoLock.setPosition(SERVO_LOCKED);
        }

        if (gamepad2.x){
            servoDepotDrop.setPosition(DEPOT_DOWN);
        }
        else {
            servoDepotDrop.setPosition(DEPOT_UP);
        }
        /* // Controls used to test the needed position of the servo
        if (gamepad2.x)
        {
            depot = depot + INC_VAL;
            servoDepotDrop.setPosition(depot/180);
            slp(100);
        }

        if (gamepad2.b)
        {
            if (depot > 0)
            {
                depot = depot - INC_VAL;
            }
            else
            {
                depot = 0;
            }
            servoDepotDrop.setPosition(depot/180);
            slp(100);

        }*/

        // Add telemetry for use while driving and for resetting system positions between matches
        telemetry.addData("Locked: ", locked); // Will say if the Lift is locked or not
        telemetry.addData("Lock Position:", servoLock.getPosition()); // Will tell position
        // of servoLock
        telemetry.addData("Depot Drop Position:", servoDepotDrop.getPosition()); // Will tell position
        // of servoDepot
        telemetry.addData("Lift Position", motorLift.getCurrentPosition()); // Will tell
        // position of motorLift
        telemetry.update(); // Updates telemetry

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