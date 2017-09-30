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

/* ------------------------------------------------------------------
 * This Op Mode Performs General Drive Train and Arm Manipulations
 * via manual control.
 * 	1. Drive train control via GamePad 1
 * 	2. Arm manipulation control via GamePad 2
 *
 * Control Modules
 *  1. Motor Controller Module
 *  2. Motor Controller Module
 *
 * Gamepads
 * 	1. Left/Right sticks: Tank drive operation
 * 	   "A" button: Toggle front/rear of robot
 * 	   "B" button: Cancel reverse operation (i.e., set front as front)
 *	2. Left Stick: Extend/retract arm
 *	   Right Stick: Pivot arm
 * ------------------------------------------------------------------
 */
@TeleOp(name = "WALL-ARR")
public class CyberVortexTeleoOpDemo extends CyberVortexAbstract
{
    // Set Additional Boolean Variables
    boolean bDirection; // Flags fwd/rev drive train operation (1 = Arm is front; 0 = Collection is front)
    char cDirection;

    //------------------------------------------------------------------
    // Constructor Method  (Not Used)
    //------------------------------------------------------------------
    public CyberVortexTeleoOpDemo()
    {
    }

    //------------------------------------------------------------------
    // Robot Initialization Method
    //------------------------------------------------------------------
    @Override
    public void init() {
    // Get references to dc motors and set initial motor modes and directions.
    // NOTE: Although super.init (see CyberAbstractOpMode) attempts to set all motor modes to Reset-Encoders, the
    // next set of operations in this init method overrides those mode commands before they
    // are recieved by the motor controller.
    super.init();

    // Set motor modes for desired telop operation.
    // Note: Run without encoders will allow the driver to manipulate motor power without
    //       limiting distance. Encoders can be monitored to prevent over/under travel, but
    //       overshoot may be significant. On the other hand, run to position will allow the
    // 		 driver to manipulate motor power in teleop, but travel range is limited to the
    //		 target position (i.e., max or min allowed target position.
    motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motorCollect.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motorFlicker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motorSpool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    // Start robot with the arm side of the robot designated as the front, and the collection
    // side designated as the rear.
    bDirection = true;
    cDirection = 'a';
} // End OpMode Initialization Method


    //------------------------------------------------------------------
    // Robot OpMode Loop Method
    //------------------------------------------------------------------
    @Override
    public void loop()
    {
        // LEFT/RIGHT DRIVE TRAIN TELEOP CONTROL
        // Gamepad 1 controls the drive train motors
        // If gamepad1.dpad_up is pressed, set arm as the front of the robot
        // if gamepad1.dpad_down is pressed, set collection as the front of the robot)
        // Set motor directions accordingly to compensate for selected robot direction



        super.loop();

        //if (gamepad1.dpad_up)
        //{
        //    bDirection = true; // Arm is front.
        //}
        //if (gamepad1.dpad_down) {
        //    bDirection = false; // Collection is front
        //}

        // Calculate drive train throttle and direction given selected robot direction.
        throttleDrive = -gamepad1.left_stick_y;
        directionDrive = gamepad1.left_stick_x;

        if (bDirection) // Arm is front
        {
            powerRight = throttleDrive - directionDrive;
            powerLeft = throttleDrive + directionDrive;
        }
        else  // Collection is front
        {
            powerRight = -throttleDrive - directionDrive;
            powerLeft = -throttleDrive + directionDrive;
        }

        //Collection System
        if(gamepad2.dpad_down)
        {
        cDirection = 'a';
        }
         else if (gamepad2.dpad_up)
        {
        cDirection = 'b';
        }

        if (cDirection == 'a')
        {
            motorCollect.setPower(0);
        }
        else if(cDirection == 'b')
        {
            motorCollect.setPower(1);
        }

        //Release System
        //if(gamepad2.a)
        //{
        //        releaseOut();
        //
        //}
        //else
        //{
        //    servoRelease.setPower(0);
        //}



        //if(gamepad2.y)
        //{
        //        releaseLock();
        //}
        //else
        //{
        //    servoRelease.setPower(0);
        //}


        //throttleALength = gamepad2.left_stick_y;  // For Run-to-Position, power polarity does not matter

        // Clip and scale the throttle, and then set motor power.
        throttleALength = Range.clip(throttleALength, -1, 1);
        throttleALength = (float) scaleInput(throttleALength);
        motorSpool.setPower(throttleALength);

        //  telemetry.addData("Mode Length ", motorALength.getChannelMode());
        // telemetry.addData("Mode Joint ", motorAJoint.getChannelMode());

        // Clip and scale the throttle, and then set power.
        throttleAJoint = Range.clip(throttleAJoint, -1, 1);
        throttleAJoint = (float) scaleInput(throttleAJoint);

        // Lock position values

        // Clip left and right drive motor power so they never exceed -1 thru 1.
        powerRight = Range.clip(powerRight, -1, 1);
        powerLeft = Range.clip(powerLeft, -1, 1);

        // Scale left and right drive motor power for better control at low power
        powerRight = (float) scaleInput(powerRight);
        powerLeft = (float) scaleInput(powerLeft);

        // Set drive motor power
        motorRight.setPower(powerRight);
        motorLeft.setPower(powerLeft);

        //When  is pressed on gamepad2
        //This controls the button presser.
        if (gamepad2.left_trigger > 0)
        {
            leftButtonOut();
        }

        if(gamepad2.left_bumper)
        {
            leftButtonIn();
        }
        //When B is pressed on gamepad1, call the rightButton function opposite to the current status.
        //This controls the button presser.
        if (gamepad2.right_trigger > 0)
        {
            rightButtonOut();
        }

        if (gamepad2.right_bumper)
        {
            rightButtonIn();
        }



        //When you press the right trigger,
        //It will run the launcher motor at full power

        if (gamepad2.x)
        {
            int rotateCounts = motorFlicker.getCurrentPosition() + LAUNCH_ROTATE;
            motorFlicker.setPower(1);
            motorFlicker.setTargetPosition(rotateCounts);
        }

        if(gamepad2.b)
        {
            loadBall();
        }

        //The following is feedback sent back to the driver station from the robot.
        //Used for testing and troubleshooting
        if (bDirection)
        {
            telemetry.addData("1. ", "CapMech is front");
        }
        else
        {
            telemetry.addData("1. ", "Collection is front");
        }
        telemetry.addData("2. ", "L/R DrvPwr: " + String.format("%.2f", powerLeft) + " / " + String.format("%.2f", powerRight));
        telemetry.addData("3. ", "L/R DrvPos: " + motorLeft.getCurrentPosition() + " / " + motorRight.getCurrentPosition());
        telemetry.addData("4. ", "Ly/Lx DriveSticks: " + String.format("%.2f", gamepad1.left_stick_y) + " / " + String.format("%.2f", gamepad1.left_stick_x));
        telemetry.addData("5. PB a = ", gamepad1.a + ", PB y = " + gamepad1.y);
        telemetry.addData("6. ", "Spool Pos: " + motorSpool.getCurrentPosition());
        telemetry.addData("7. ", "Collection Active?: " + cDirection);
        telemetry.addData("8. ", "Beacon Sees: ", seeBeacon(colorSensorB.red(), colorSensorB.blue(), hsvValues[0]));
        telemetry.addData("9. ", "Line Seen?: ", seeLine(colorSensorL.red(),colorSensorL.blue(),colorSensorL.green(), hsvValues[0],colorSensorL.alpha()));
        telemetry.addData("10. ", "Right Pusher Pos:" + rightOut);
        telemetry.addData("11. ", "Left Pusher Pos:" + leftOut);

    } // End OpMode Loop Method

    @Override
    public void stop ()
    {
            super.stop();
    }

} // End OpMode


