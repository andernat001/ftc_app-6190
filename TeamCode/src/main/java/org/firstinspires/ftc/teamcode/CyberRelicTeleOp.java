/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.Range;
        import static android.os.SystemClock.sleep;
        import com.qualcomm.hardware.bosch.BNO055IMU;

        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "RelicBot")
public class CyberRelicTeleOp extends CyberRelicAbstract {
    public CyberRelicTeleOp() {
    }

    @Override
    public void init() {

        super.init();

        // Set all motors to run without encoders
        motorRightA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGlyphLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fieldOrient = false;
        bDirection = true;
        grabbed = false;
        //lightSensor.enableLed(false);

        /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);*/
    }

    @Override
    public void loop() {

        super.loop();

        // Set drive motor power
        motorRightA.setPower(powerRightA);
        motorRightB.setPower(powerRightB);
        motorLeftA.setPower(powerLeftA);
        motorLeftB.setPower(powerLeftB);

        // Set controls
        velocityDrive = gamepad1.left_stick_y;
        strafeDrive = gamepad1.left_stick_x;
        rotationDrive = gamepad1.right_stick_x;

        //Field-Oriented drive code
        if (gamepad1.dpad_up)
        {
            fieldOrient = true;
        }
        if (gamepad1.dpad_down)
        {
            fieldOrient = false;
        }

        //Set doubles x,y,and gyro
        x = strafeDrive;
        y = velocityDrive;
        gyro = gyroSensor.getHeading(); //(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //Field-Oriented drive Algorithm
        if (fieldOrient)
        {
            temp = y * Math.cos(gyro) + x * Math.sin(gyro);
            x = -y * Math.sin(gyro) + x * Math.cos(gyro);
            y = temp;
        }
        telemetry.addData("Strafe:", + strafeDrive);
        telemetry.addData("Velocity:", + velocityDrive);
        telemetry.update();

        //Set floats strafeDrive and velocityDrive
        strafeDrive = (float) x;
        velocityDrive = (float) y;

        // Scale drive motor power for better control at low power
        powerRightA = (float) scaleInput(powerRightA);
        powerRightB = (float) scaleInput(powerRightB);
        powerLeftA = (float) scaleInput(powerLeftA);
        powerLeftB = (float) scaleInput(powerLeftB);

        if (bDirection) // Glyph is front
        {
            powerRightA = velocityDrive + rotationDrive + strafeDrive;
            powerRightB = velocityDrive + rotationDrive - strafeDrive;
            powerLeftA = velocityDrive - rotationDrive - strafeDrive;
            powerLeftB = velocityDrive - rotationDrive + strafeDrive;
        } else  // Relic is front
        {
            powerRightA = velocityDrive - rotationDrive - strafeDrive;
            powerRightB = velocityDrive - rotationDrive + strafeDrive;
            powerLeftA = velocityDrive + rotationDrive + strafeDrive;
            powerLeftB = velocityDrive + rotationDrive - strafeDrive;
        }

        //Create dead-zone for drive train controls
        if (gamepad1.left_stick_x <= 0.05 && gamepad1.left_stick_x >= -0.05)
        {
            gamepad1.left_stick_x = 0;
        }

        if (gamepad1.left_stick_y <= 0.05 && gamepad1.left_stick_y >= -0.05)
        {
            gamepad1.left_stick_y = 0;
        }

        if (gamepad1.right_stick_x <= 0.05 && gamepad1.right_stick_x >= -0.05)
        {
            gamepad1.right_stick_x = 0;
        }

        // If the left stick and the right stick is used it halves the power  of the motors for better accuracy
        if (gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0 && gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0)
        {


            powerRightA = Range.clip(powerRightA, -0.5f, 0.5f);
            powerRightB = Range.clip(powerRightB, -0.5f, 0.5f);
            powerLeftA = Range.clip(powerLeftA, -0.5f, 0.5f);
            powerLeftB = Range.clip(powerLeftB, -0.5f, 0.5f);

        } else if (gamepad1.left_stick_x > 0 || gamepad1.left_stick_x < 0 && gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0)
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

        //Change direction that is front for the robot
        if (gamepad1.dpad_left)
        {
            bDirection = true; // Arm is front.
        }
        if (gamepad1.dpad_right)
        {
            bDirection = false; // Collection is front
        }

        if (fieldOrient)
        {
            bDirection = true;
        }


        //Controls for grabbing the glyph
        if (gamepad2.x && !grabbed)
        {
            grabbed = true;
        }
        else if (gamepad2.x && grabbed)
        {
            grabbed = false;
        }

        if (grabbed)
        {
            servoGlyph1.setPosition(1);
            servoGlyph2.setPosition(0.60);
        }
        if (!grabbed)
        {
            servoGlyph1.setPosition(0.83);
            servoGlyph2.setPosition(0.77);
        }
/*
        if (gamepad2.y)
        {
            glyph1 = glyph1 + INC_VAL;
            servoGlyph1.setPosition(glyph1/180);
            sleep(100);

        }

        if (gamepad2.a)
        {
            if (glyph1 > 0)
            {
                glyph1 = glyph1 - INC_VAL;
            }
            else
            {
                glyph1 = 0;
            }
            servoGlyph1.setPosition(glyph1/180);
            sleep(100);

        }
        if (gamepad2.x)
        {
            if (glyph2 < 180)
            {
                glyph2 = glyph2 + INC_VAL;
            }
            else
            {
                glyph1 = 180;
            }
            servoGlyph2.setPosition(glyph2/180);
            sleep(100);

        }

        if (gamepad2.b)
        {
            glyph2 = glyph2 - INC_VAL;
            servoGlyph2.setPosition(glyph2/180);
            sleep(100);

        }

*/

        if(motorGlyphLift.getCurrentPosition() >= -7150){

            //Controls for lifting the glyph
            //Set controls for lift
            throttleLift = gamepad2.left_stick_y;
            // Clip and scale the throttle, and then set motor power.
            throttleLift = Range.clip(throttleLift, -1, 1);
            throttleLift = (float) scaleInput(throttleLift);
            motorGlyphLift.setPower(throttleLift);

            //Create dead-zone for lift control
            if (gamepad2.left_stick_y <= 0.05 && gamepad2.left_stick_y >= -0.05)
            {
                gamepad2.left_stick_y = 0;
            }
        }
        else {
            motorGlyphLift.setTargetPosition(-7145);
            motorGlyphLift.setPower(.1);
        }




        if(gamepad2.left_trigger != 0){
        telemetry.addData("Lift", motorGlyphLift.getCurrentPosition());
        telemetry.addData("Glyph1", servoGlyph1.getPosition());
        telemetry.addData("Glyph2", servoGlyph2.getPosition());
        telemetry.update();}


// End OpMode Loop Method
    }
    @Override
    public void stop ()
    {
        super.stop();
    }
}
