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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

@TeleOp(name="Panic: Teleop", group="Panic")
public class PanicTeleop extends PanicAbstractTeleop {

	/**
	 * Constructor
	 */
	public PanicTeleop() {

	}

	private float left = 0.0f;
	private float right = 0.0f;
	private boolean shooterIsForward = true;
	private boolean lastForwardState = false;

	@Override
	public void loop() {

		// let the driver change the forward direction on the robot
		if (gamepad1.y && !lastForwardState && shooterIsForward)
		{
			buttonPusherIsForward();
			shooterIsForward = false;
		}
		else if (gamepad1.y && !lastForwardState && !shooterIsForward)
		{
			shooterIsForward();
			shooterIsForward = true;
		}
		lastForwardState = gamepad1.y;

		// Tank drive
		// Note that if y equal -1 then joystick is pushed all of the way forward.
		if (shooterIsForward)
		{
			left = -gamepad1.left_stick_y;
			right = -gamepad1.right_stick_y;
		}
		else
		{
			right = -gamepad1.left_stick_y;
			left = -gamepad1.right_stick_y;

		}

		// clip the right/left values so that the values never exceed +/- 1
		right = Range.clip(right, -1, 1);
		left = Range.clip(left, -1, 1);

		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		right = (float) scaleInput(right * 0.75);
		left = (float) scaleInput(left * 0.75);

		// write the values to the motors
		motorRight.setPower(right);
		motorLeft.setPower(left);

		// Uncomment me if we want to test the amount of time needed for the button pusher
//		if(gamepad2.dpad_left && !lastDpadLeft) {
//			changeButtonSpeed( -1 );
//		}
//		else if (gamepad2.dpad_right && !lastDpadRight) {
//			changeButtonSpeed( 1 );
//		}

        // button pusher
		if (gamepad2.left_bumper && !lastLeftBumper) {
			leftButton();
		} else if (gamepad2.right_bumper && !lastRightBumper) {
			rightButton();
		}
		lastLeftBumper = gamepad2.left_bumper;
		lastRightBumper = gamepad2.right_bumper;

		// collection system toggle
        if (gamepad2.a) {
			collectionMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            collectionMotor.setPower(1);
        }

		// if we use y then we break the zip ties
//		if (gamepad2.y) {
//			collectionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//			collectionMotor.setPower(1);
//		}

        if (gamepad2.b) {
            collectionMotor.setPower(0);
        }

		if (gamepad2.left_trigger >= 0.75) {
			startFlywheel();
		}
		else {
			stopFlywheel();
		}

        if(gamepad2.dpad_up && !lastDpadUp && flywheelRunning) // thx highschoolers for this!
        {
            flywheelSpeed = flywheelSpeed + 0.05;
            flywheelSpeed = Range.clip(flywheelSpeed, 0.25, 0.60); // Now it will actually clip!
            setFlywheelSpeed(flywheelSpeed);
        }
        lastDpadUp = gamepad2.dpad_up;

        if(gamepad2.dpad_down && !lastDpadDown && flywheelRunning) // and this!
        {
			flywheelSpeed = flywheelSpeed - 0.05;
            flywheelSpeed = Range.clip(flywheelSpeed, 0.25, 0.60); // Now it will actually clip!
            setFlywheelSpeed(flywheelSpeed);
        }
        lastDpadDown = gamepad2.dpad_down;

        if (gamepad2.right_trigger >= 0.75) {
            particleFlipper.setPosition(0.65);
        }
        else {
            particleFlipper.setPosition(0.46);
        }

		telemetry.addData("Flywheel Speed Power", flywheelSpeed); // telemetry stuff
//		telemetry.addData("Voltage", voltageSensor.getVoltage());
		telemetry.addData("Beacon Sensor", beaconSensor.red() + "/" + beaconSensor.blue() + "/" + beaconSensor.green());
		telemetry.addData("Beacon Sensor 2", beaconSensor2.red() + "/" + beaconSensor2.blue() + "/" + beaconSensor2.green());
		telemetry.update();
	}



    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
	@Override
	public void stop() {
		System.gc();
	}
}
