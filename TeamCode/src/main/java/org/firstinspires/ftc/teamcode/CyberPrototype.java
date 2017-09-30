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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class CyberPrototype extends OpMode
{


	// TETRIX VALUES.
	//final static double FLIPPER_MIN_RANGE = 0;
	//final static double FLIPPER_MAX_RANGE = .1;
	//double flipDelta;

	//Drive Train Names
	DcMotor motorRight;
	DcMotor motorLeft;
	//DcMotor motorBRight;
	//DcMotor motorBLeft;
	//Servo servoLock1;
	//Servo servoLock2
	//Arm Names
	DcMotor motorALength;
	DcMotor motorAJoint;

	final static int
		LIMIT_ARM_EXT_MAX = 1000000,
		LIMIT_ARM_EXT_MIN = 0,
		LIMIT_ARM_PIV_MAX = 0,
		LIMIT_ARM_PIV_MIN = -1680;

	//Ejection System Names
	//DcMotor motorEjection;
	//Servo servoFlipper;

	//Variables for Direction Flip-Flop
	boolean bDirection;
	float right, left;

	//Lock Status
	boolean lStatus;

	public CyberPrototype()
	{

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init()
	{


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		//Collection and Ejection System



		//Arm
		motorAJoint = hardwareMap.dcMotor.get("joint");
		motorAJoint.setDirection(DcMotor.Direction.FORWARD);
		motorAJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		motorALength = hardwareMap.dcMotor.get("length");
		motorALength.setDirection(DcMotor.Direction.REVERSE);
		motorALength.setMode(DcMotor.RunMode.RUN_TO_POSITION);


		//Drive Train
		motorRight = hardwareMap.dcMotor.get("right");
		motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motorRight.setDirection(DcMotor.Direction.FORWARD);



		motorLeft = hardwareMap.dcMotor.get("left");
		motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motorLeft.setDirection(DcMotor.Direction.FORWARD);

		//servoLock1 = hardwareMap.servo.get("lock1");
		//servoLock2 = hardwareMap.servo.get("lock2");

		//Default Direction (true=Claw side)
        bDirection = true;

		//Default lock position
		//lStatus = true;


	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop()
	{

		//GamePad 1, Driver

		//Declaring powers and their controls
		float throttle = -gamepad1.left_stick_y;
		float direction = gamepad1.left_stick_x;



		//This set-up allows the front and back
		//sides of the robot to be flip-flopped.

		if (gamepad1.dpad_up)
		{
			bDirection = true;
		}
		if (gamepad1.dpad_down)
		{
			bDirection = false;
		}

		 //Claw side=front
		 if (bDirection)
		 {
			 right = throttle - direction;
			 left = throttle + direction;
			 motorLeft.setDirection(DcMotor.Direction.REVERSE);
			 motorRight.setDirection(DcMotor.Direction.FORWARD);
		 }
		 //Collection side=front
		 else
		 {
			 right = throttle + direction;
			 left = throttle - direction;
			 motorRight.setDirection(DcMotor.Direction.REVERSE);
			 motorLeft.setDirection(DcMotor.Direction.FORWARD);
		 }

		 if (gamepad1.y)
		 {
			lStatus = false;
		 }
		 if (gamepad1.a)
		 {
			 lStatus = true;
		 }

		// clip the right/left values so that the values never exceed +/- 1
		right = Range.clip(right, -1, 1);
		left = Range.clip(left, -1, 1);

		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		right = (float)scaleInput(right);
		left =  (float)scaleInput(left);

//		if (lStatus)
//		{
//			servoLock1.setPosition(.9);
// 			servoLock2,setPosition(.9);
//		}
//		else
//		{
//			servoLock1.setPosition(0);
//			servoLock2.setPosition(0);
//		}

		//if (gamepad1.x) {}


		// write the values to the motors
		motorRight.setPower(right);
		//motorBRight.setPower(right);
		motorLeft.setPower(left);
		//motorBLeft.setPower(left);
        //motorEjection.setPower(ejection);



		//GamePad 2, Operator
		float ExtendSpeed;
		 if (gamepad2.left_stick_y >= 0)
		{
			motorALength.setTargetPosition(LIMIT_ARM_EXT_MAX);
			ExtendSpeed = gamepad2.left_stick_y;
		}
		else
		{
			motorALength.setTargetPosition(LIMIT_ARM_EXT_MIN);
			ExtendSpeed = -gamepad2.left_stick_y;
		}

		float JointAngle;
		if (gamepad2.right_stick_y >= 0)
		{
			motorAJoint.setTargetPosition(LIMIT_ARM_PIV_MAX);
			JointAngle = gamepad2.right_stick_y;
		}
		else
		{
			motorAJoint.setTargetPosition(LIMIT_ARM_PIV_MIN);
			JointAngle = -gamepad2.right_stick_y;
		}
		// Power will never exceed +/- 1
		ExtendSpeed = Range.clip(ExtendSpeed, -1, 1);
		JointAngle = Range.clip(JointAngle, -1, 1);

		//Arms are scaled for easier control
		ExtendSpeed = (float)scaleInput(ExtendSpeed);
		JointAngle =  (float)scaleInput(JointAngle);


		//Set power to motors
		motorALength.setPower(ExtendSpeed);
		motorAJoint.setPower(JointAngle);



		//Telemetry FeedBack
        telemetry.addData("Text", "*** Robot Data***");

		telemetry.addData("Extend ", motorALength.getCurrentPosition());
		telemetry.addData("ExtendPower ",ExtendSpeed);

		//Alerting drivers to the current forward direction
		if (bDirection) telemetry.addData("Direction", "Arm");
		else telemetry.addData("Direction", "Collection");

        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
		telemetry.addData("Debug Arm1","Encoders L/J = " + motorALength.getCurrentPosition() + "/" + motorAJoint.getCurrentPosition());
		telemetry.addData("Debug Arm2","Target L/J = " + motorALength.getTargetPosition() + "/" + motorAJoint.getTargetPosition());
		telemetry.addData("Debug Arm3","Target L/J = " + motorALength.getTargetPosition() + "/" + motorAJoint.getTargetPosition());
	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}

    	
	/*
	 * This method scales the joystick input so for low joystick values, the 
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds, as well as control the arm
	 * with better accuracy.
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
		double dScale = 0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		}
		else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}
	/*
     * This method allows a power value to be limited
     * by observing the position of the encoders. The
     * function asks for the power value to be limited,
     * the position of the motor driving it, and a
     * lower and upper limit. This allows it be used for
     * multiple power values at once. In this case it
     * has been used for both the length of the arm
     * and the angle.
	 */
	float Limit(float powerValue,double currentPos, double lowerLimit, double upperLimit)  {
		if (currentPos > upperLimit) {
			if (powerValue > 0) powerValue = 0;
		}
		if (currentPos < lowerLimit) {
			if (powerValue < 0) powerValue = 0;
		}
		return powerValue;
	}
}
