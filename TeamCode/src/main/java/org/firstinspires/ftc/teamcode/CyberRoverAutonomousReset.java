package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Rover Reset", group = "RiderModes")
public class CyberRoverAutonomousReset extends CyberRoverAbstract{

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
                    motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    timer.reset();
                    seqRobot++;
                    break;
                }

                case 2: { // Reset the Lift's position and lock the locking mechanism
                    motorLift.setTargetPosition(LIFT_DOWN);
                    motorLift.setPower(0.35);

                    if (timer.milliseconds() > 2000) // After the lift has run for 2 seconds, lock
                        // the servo
                    {
                        servoLock.setPosition(SERVO_LOCKED);
                        timer.reset();
                        seqRobot++;
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
            telemetry.addData("Lift Encoder: ", motorLift.getCurrentPosition());
            telemetry.addData("Locked: ", locked); // Will say "true" or "false"
            telemetry.update();


        }

}
