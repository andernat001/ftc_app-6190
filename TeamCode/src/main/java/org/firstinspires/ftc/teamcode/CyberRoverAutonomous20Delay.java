package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Rover20", group = "RiderModes")
public class CyberRoverAutonomous20Delay extends CyberRoverAbstract{

    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);  // Added so
    // opMode does not sleep

    //------------------------------------------------------------------
    // Robot OpMode Loop Method
    //------------------------------------------------------------------

    @Override
    public void init() {
        super.init();

        motorLeftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    @Override
    public void loop() {

        // START ROBOT SEQUENCE
        // Establish the robot sequence of operation with the Switch operation.
        // The active Case (i.e., sequence step) is established by the value in seqRobot.
        // After a Case executes, Break the switch to prevent executing subsequent cases
        // unintentionally.
        switch (seqRobot) {
            case 0: {
                timer.reset();
                seqRobot++;
                break;
            }

            case 4:
            case 3:
            case 2:
            case 1: { // Set all motor power to 0
                motorLift.setPower(0);
                motorRightA.setPower(0);
                motorRightB.setPower(0);
                motorLeftA.setPower(0);
                motorLeftB.setPower(0);
                if (timer.milliseconds() > 5000) // Wait 20 seconds (total)
                {
                    seqRobot++;
                    timer.reset();
                }
                break;
            }

            case 5: { // Unlock the locking mechanism and set down robot
                servoLock.setPosition(SERVO_UNLOCKED);// Unlock locking mechanism
                if (timer.milliseconds() > 2000) // Wait 1.500 seconds
                {
                    motorLift.setTargetPosition(LIFT_UP);
                    motorLift.setPower(0.35);
                }
                if (motorLift.getCurrentPosition() <= LIFT_UP + 20 &&
                        motorLift.getCurrentPosition() >= LIFT_UP - 20) // Once motorLift is
                // within 20 encoder counts of LIFT_UP the autonomous will continue
                {
                    seqRobot++;
                }
                break;
            }

            case 6: { // Check the position of the motor and don't move on until it is within
                // the correct range
                motorLift.setTargetPosition(LIFT_UP);
                motorLift.setPower(0.05);
                if (motorLift.getCurrentPosition() <= LIFT_UP + 10 &&
                        motorLift.getCurrentPosition() >= LIFT_UP - 10) {
                    motorLift.setPower(0);
                    seqRobot++;
                } else { // If not within range, repeat case
                    seqRobot = 6;
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
        }    // End Robot Sequence

        // Adds telemetry for trouble-shooting autonomous
        telemetry.addData("Lift Target: ", motorLift.getTargetPosition());
        telemetry.addData("Lift Encoder: ", motorLift.getCurrentPosition());
        telemetry.addData("LeftA Encoder: ", motorLeftA.getCurrentPosition());
        telemetry.addData("LeftA Target ", motorLeftA.getTargetPosition());
        telemetry.addData("LeftB Encoder: ", motorLeftB.getCurrentPosition());
        telemetry.addData("LeftB Target: ", motorLeftB.getTargetPosition());
        telemetry.addData("LeftB Power: ", motorLeftB.getPower());
        telemetry.addData("RightA Encoder: ", motorRightA.getCurrentPosition());
        telemetry.addData("RightA Target: ", motorRightA.getTargetPosition());
        telemetry.addData("RightB Encoder: ", motorRightB.getCurrentPosition());
        telemetry.addData("RightB Target: ", motorRightB.getTargetPosition());
        telemetry.addData("RightB Power: ", motorRightB.getPower());
        telemetry.addData("Case: ", seqRobot);

        telemetry.update();


    }

} // End
