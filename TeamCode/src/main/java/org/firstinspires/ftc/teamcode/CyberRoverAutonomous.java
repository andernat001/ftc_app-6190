package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RoverO", group = "RiderModes")
public class CyberRoverAutonomous extends CyberRoverAbstract{

    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);  // Added so
    // opMode does not sleep

    //------------------------------------------------------------------
    // Robot OpMode Loop Method
    //------------------------------------------------------------------

    @Override
    public void init() {

    }

    @Override
    public void loop() {

        // START ROBOT SEQUENCE
        // Establish the robot sequence of operation with the Switch operation.
        // The active Case (i.e., sequence step) is established by the value in seqRobot.
        // After a Case executes, Break the switch to prevent executing subsequent cases unintentionally.
        switch (seqRobot) {
                /*case 1: { // Set all motor power to 0
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
                    if (motorLift.getCurrentPosition() <= LIFT_UP + 20 &&
                            motorLift.getCurrentPosition() >= LIFT_UP -20) // Once motorLift is
                        // within 20 encoder counts of LIFT_UP the autonomous will continue
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
                        motorLift.setTargetPosition(LIFT_UP);
                        motorLift.setPower(0.05);
                        seqRobot++;
                        timer.reset();
                        break;
                    } else { // If not within range, repeat case
                        seqRobot = 3;
                        timer.reset();
                    }
                    break;
                }*/

            case 4: { // Strafe the robot to the left and off the shuttle

                // Define drive train target position and motor power.E
                targetDrRotateDeg = 0f; // Not used in this step, but reported via telemetry in the next step.
                targetDrDistInch = 5f; // Set target distance
                targetPower = 0.05d;  // Set power

                // Use this OpModes's custom cmdMoveR method to calculate new target (in encoder
                // counts) and to initiate the move. cmdMoveR initiates a relative move.
                // cmdMove Parameters (distance inches, encoder count per inch, power, motor).
                targetPosLeftA = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveR(-targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveR(-targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);


                    /*if (timer.milliseconds() < 1250) // Strafe for 1.250 seconds
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
                    }*/
                seqRobot++;
                break;
            }

            case 5:
                // Hold until drive train move is complete
            {
                // Use this OpModes's custom chkMove to determine if motor move(s) are complete
                // chkMove Parameters (motor, target, allowed +/- error counts from target)
                // May need to add motor-is-busy check to ensure electric breaking complete.
                // May need to compensate for motor power if one motor is faster than another to keep straight line.
                if (chkMove(motorLeftA, targetPosLeftA, ERROR_DRV_POS) &&
                        chkMove(motorLeftB, -targetPosLeftB, ERROR_DRV_POS) &&
                        chkMove(motorRightA, -targetPosRightA, ERROR_DRV_POS) &&
                        chkMove(motorRightB, targetPosRightB, ERROR_DRV_POS)) {    // If drive train at target, hold position for X amount of time to stabilize motors.
                    if (seqRobot == 5) {
                        seqRobot++;
                    }else {
                        seqRobot = 99;
                    }
                }
                break;
            }

            case 6:// Reset encoders
            {
                motorLeftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftA.setTargetPosition(0);
                motorLeftB.setTargetPosition(0);
                motorRightA.setTargetPosition(0);
                motorRightB.setTargetPosition(0);
                seqRobot++;
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
        //telemetry.addData("Lift Target: ", motorLift.getTargetPosition());
        //telemetry.addData("Lift Encoder: ", motorLift.getCurrentPosition());
        telemetry.addData("LeftA Encoder: ", motorLeftA.getCurrentPosition());
        telemetry.addData("LeftA Target ", motorLeftA.getTargetPosition());
        telemetry.addData("LeftB Encoder: ", motorLeftB.getCurrentPosition());
        telemetry.addData("LeftB Target: ", motorLeftB.getTargetPosition());
        telemetry.addData("RightA Encoder: ", motorRightA.getCurrentPosition());
        telemetry.addData("RightA Target: ", motorRightA.getTargetPosition());
        telemetry.addData("RightB Encoder: ", motorRightB.getCurrentPosition());
        telemetry.addData("RightB Target: ", motorRightB.getTargetPosition());

        //telemetry.addData("Lift Power: ", motorLift.getPower());
        //telemetry.addData("Lock_Position: ", servoLock.getPosition());
        telemetry.addData("Case: ", seqRobot);
        telemetry.update();


    }

} // End OpMode