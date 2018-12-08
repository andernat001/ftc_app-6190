package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Rover", group = "RiderModes")
public class CyberRoverAutonomous extends CyberRoverAbstract{

    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);  // Added so
    // opMode does not sleep

        //------------------------------------------------------------------
        // Robot OpMode Loop Method
        //------------------------------------------------------------------

        @Override
        public void init() {

            super.init();

        }

        @Override
        public void loop() {

            super.loop();

            // START ROBOT SEQUENCE
            // Establish the robot sequence of operation with the Switch operation.
            // The active Case (i.e., sequence step) is established by the value in seqRobot.
            // After a Case executes, Break the switch to prevent executing subsequent cases unintentionally.
            switch (seqRobot) {
                case 1: { // Set all motor power to 0
                    motorLift.setPower(0);
                    motorRightA.setPower(0);
                    motorRightB.setPower(0);
                    motorLeftA.setPower(0);
                    motorLeftB.setPower(0);

                    seqRobot++;
                    timer.reset();
                    break;
                }

                case 2: // Transition lift motor to Run-to-Position mode.
                {
                    motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    seqRobot++;
                    timer.reset();
                    break;  // Note: The Break command will ensure one scan of code executed while motor
                    // modes take effect. If Run-to-Position not enabled, unexpected operations
                    // may occur.
                }

                case 3: { // Unlock the locking mechanism and set down robot
                    servoLock.setPosition(SERVO_UNLOCKED);
                    motorLift.setTargetPosition(LIFT_UP);
                    motorLift.setPower(0.075);

                    seqRobot++;
                    timer.reset();
                    break;
                }

                case 4: { // Check the position of the motor and don't move on until it is within
                          // the correct range
                    if (motorLift.getCurrentPosition() >= LIFT_UP - 10 &&
                            motorLift.getCurrentPosition() <= LIFT_UP) {
                        seqRobot++;
                        timer.reset();
                        break;
                    } else {
                        seqRobot = 4;
                        timer.reset();
                        break;
                    }

                }

                case 5: { // Strafe the robot to the left and off the shuttle
                    if (timer.milliseconds() < 1500) {
                        motorLeftA.setPower(0.1);
                        motorLeftB.setPower(-0.09);
                        motorRightA.setPower(-0.09);
                        motorRightB.setPower(0.1);
                    } else {
                        motorLeftA.setPower(0);
                        motorLeftB.setPower(0);
                        motorRightA.setPower(0);
                        motorRightB.setPower(0);
                    }
                }
                case 99:  // Done
                {
                    break;
                }

                default: {
                    break;
                }
                // NOTE - By stringing many cases together a complex operation can be built.

            }    // End Robot Sequence

            // Adds telemetry for trouble-shooting autonomous
            telemetry.addData("Lift Encoder: ", motorLift.getCurrentPosition());
            telemetry.addData("Lock_Position: ", servoLock.getPosition());
            telemetry.addData("Case: ", seqRobot);
            telemetry.update();


        }

    } // End OpMode
