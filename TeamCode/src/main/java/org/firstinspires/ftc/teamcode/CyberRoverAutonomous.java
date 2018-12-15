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

                    if (timer.milliseconds() > 1000) // Wait 1 second
                    {
                        seqRobot++;
                        timer.reset();
                    }
                    break; // Note: The Break command will ensure one scan of code executed while motor
                    // modes take effect. If Run-to-Position not enabled, unexpected operations
                    // may occur.
                }

                case 2: { // Unlock the locking mechanism and set down robot
                    servoLock.setPosition(SERVO_UNLOCKED);// Unlock locking mechanism

                    if (timer.milliseconds() > 1250) // Wait 1.250 seconds
                    {
                        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set down robot
                        motorLift.setTargetPosition(LIFT_UP);
                        motorLift.setPower(0.15);
                    }
                    if (motorLift.getCurrentPosition() <= LIFT_UP + 10 &&
                            motorLift.getCurrentPosition() >= LIFT_UP -10) // Once motorLift is
                        // within 10 encoder counts of LIFT_UP the autonomous will continue
                    {
                        seqRobot++;
                        timer.reset();
                    }
                    break;
                }

                case 3: { // Check the position of the motor and don't move on until it is within
                          // the correct range
                    if (motorLift.getCurrentPosition() >= LIFT_UP - 10 &&
                            motorLift.getCurrentPosition() <= LIFT_UP + 10) {
                        seqRobot++;
                        timer.reset();
                        break;
                    } else { // If not within range, repeat case
                        seqRobot = 3;
                        timer.reset();
                    }
                    break;
                }

                case 4: { // Strafe the robot to the left and off the shuttle
                    if (timer.milliseconds() < 1250) // Strafe for 1.250 seconds
                    {
                        motorLeftA.setPower(0.1);
                        motorLeftB.setPower(-0.1);
                        motorRightA.setPower(-0.1);
                        motorRightB.setPower(0.1);
                    } else { // Once time on this case passes 1.250 seconds all motors will set
                        // power to 0 and will continue
                        motorLeftA.setPower(0);
                        motorLeftB.setPower(0);
                        motorRightA.setPower(0);
                        motorRightB.setPower(0);
                        seqRobot++;
                        timer.reset();
                    }
                    break;
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
            telemetry.addData("Lift Target: ", motorLift.getTargetPosition());
            telemetry.addData("Lift Encoder: ", motorLift.getCurrentPosition());
            telemetry.addData("Lift Power: ", motorLift.getPower());
            telemetry.addData("Lock_Position: ", servoLock.getPosition());
            telemetry.addData("Case: ", seqRobot);
            telemetry.update();


        }

    } // End OpMode
