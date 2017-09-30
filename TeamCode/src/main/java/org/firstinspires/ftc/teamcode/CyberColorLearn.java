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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/* ------------------------------------------------------------------
 * This Op Mode is used to measure color while the robot is moving.
 * ------------------------------------------------------------------
 */

public class CyberColorLearn extends CyberAbstractOpMode
{
    //------------------------------------------------------------------
    // Robot OpMode Loop Method
    //------------------------------------------------------------------

    @Override
    public void init() {

        super.init();


    }

    @Override
    public void loop()
    {

        super.loop();

        // START ROBOT SEQUENCE
        // Establish the robot sequence of operation with the Switch operation.
        // The active Case (i.e., sequence step) is established by the value in seqRobot.
        // After a Case executes, Break the switch to prevent executing subsequent cases unintentionally.
        switch (seqRobot)
        {

            case 1:
            {
                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                seqRobot++;
                break;
            }

            case 2:
            {
                if (gamepad1.a) seqRobot++;
                break;
            }

            case 3:
            {
                motorRight.setPower(0.1);
                motorLeft.setPower(0.1);
                if (!gamepad1.a) seqRobot++;
                break;
            }

            case 4:
                if (gamepad1.a) seqRobot++;
                break;

            case 5:
            {
                motorRight.setPower(0);
                motorLeft.setPower(0);
                if (!gamepad1.a) seqRobot = 2;
                break;
            }

        }  // End seqRobot
        // NOTE - By stringing many cases together a complex operation can be built.


        telemetry.addData("1. ", "Seq #  : " + seqRobot);
        telemetry.addData("2. ", "Drive Pos L/R" + motorLeft.getCurrentPosition() + "/" + motorRight.getCurrentPosition());

        telemetry.addData("3. Color Values","");
        telemetry.addData("4. Hue", hsvValues[0]);
        telemetry.addData("5. Sat", hsvValues[1]);
        telemetry.addData("6. V", hsvValues[2]);
        telemetry.addData("7. Current color Blue?", blue);
        telemetry.addData("8. Current color Red?", red);
        telemetry.addData("9. Current color White?", white);

    } // End OpMode Loop Method

} // End OpMode
