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

@TeleOp(name = "RelicBot")
public class CyberRelicTeleoOp extends CyberRelicAbstract
{
    public CyberRelicTeleoOp()
    {
    }

    @Override
    public void init()
    {

        super.init();

        // Set all motors to run without encoders
        motorRightA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop()
    {

        super.loop();

        // Set drive motor power
        motorRightA.setPower(powerRightA);
        motorRightB.setPower(powerRightB);
        motorLeftA.setPower(powerLeftA);
        motorLeftB.setPower(powerLeftB);

        // Set controls
        velocityDrive = -gamepad1.left_stick_y;
        strafeDrive = gamepad1.left_stick_x;
        rotationDrive = gamepad1.right_stick_x;

        // Set algorithm
        powerRightA = velocityDrive + rotationDrive + strafeDrive;
        powerRightB = velocityDrive + rotationDrive - strafeDrive;
        powerLeftA = velocityDrive - rotationDrive - strafeDrive;
        powerLeftB = velocityDrive - rotationDrive + strafeDrive;

        // Scale drive motor power for better control at low power
        powerRightA = (float) scaleInput(powerRightA);
        powerRightB = (float) scaleInput(powerRightB);
        powerLeftA = (float) scaleInput(powerLeftA);
        powerLeftB = (float) scaleInput(powerLeftB);

        // Round the joystick's value when between 0.05 and -0.05 to 0
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

        // If the left stick and the right stick is used it halves the power  of the motors
        if (gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0 && gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0)
        {


            powerRightA = Range.clip(powerRightA, -0.5f, 0.5f);
            powerRightB = Range.clip(powerRightB, -0.5f, 0.5f);
            powerLeftA = Range.clip(powerLeftA, -0.5f, 0.5f);
            powerLeftB = Range.clip(powerLeftB, -0.5f, 0.5f);

        }
        else if (gamepad1.left_stick_x > 0 || gamepad1.left_stick_x < 0 && gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0)
        {

            powerRightA = Range.clip(powerRightA, -0.5f, 0.5f);
            powerRightB = Range.clip(powerRightB, -0.5f, 0.5f);
            powerLeftA = Range.clip(powerLeftA, -0.5f, 0.5f);
            powerLeftB = Range.clip(powerLeftB, -0.5f, 0.5f);

        }
        else
        {

            powerRightA = Range.clip(powerRightA, -1, 1);
            powerRightB = Range.clip(powerRightB, -1, 1);
            powerLeftA = Range.clip(powerLeftA, -1, 1);
            powerLeftB = Range.clip(powerLeftB, -1, 1);

        }

        if(gamepad1.dpad_up)
        {
            fieldOrient = true;
        }

        if(gamepad1.dpad_down)
        {
            fieldOrient = false;
        }

        if(fieldOrient)
        {
            temp = y * Math.cos(gyro) + x * Math.sin(gyro);
            x = -y * Math.sin(gyro) + x * Math.cos(gyro);
            y = temp;
        }

    }// End OpMode Loop Method

    @Override
    public void stop ()
    {
        super.stop();
    }





}
