package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Rover10", group = "RiderModes")
public class CyberRoverAutonomous10Delay extends CyberRoverAbstract{

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

            case 2:
            case 1: { // Set all motor power to 0
                motorLift.setPower(0);
                motorRightA.setPower(0);
                motorRightB.setPower(0);
                motorLeftA.setPower(0);
                motorLeftB.setPower(0);
                if (timer.milliseconds() > 5000) // Wait 10 seconds (total)
                {
                    seqRobot++;
                    timer.reset();
                }
                break;
            }

            case 3: { // Unlock the locking mechanism and set down robot
                servoLock.setPosition(SERVO_UNLOCKED);// Unlock locking mechanism
                if (timer.milliseconds() > 2000) // Wait 1.500 seconds
                {
                    motorLift.setTargetPosition(LIFT_UP);
                    motorLift.setPower(0.35);
                }
                if (motorLift.getCurrentPosition() <= LIFT_UP + 20 &&
                        motorLift.getCurrentPosition() >= LIFT_UP -20) // Once motorLift is
                // within 20 encoder counts of LIFT_UP the autonomous will continue
                {
                    seqRobot++;
                }
                break;
            }

            case 4: { // Check the position of the motor and don't move on until it is within
                // the correct range
                motorLift.setTargetPosition(LIFT_UP);
                motorLift.setPower(0.15);
                if (motorLift.getCurrentPosition() >= LIFT_UP - 10 &&
                        motorLift.getCurrentPosition() <= LIFT_UP + 10) {
                    motorLift.setPower(0);
                    seqRobot++;
                } else { // If not within range, repeat case
                    seqRobot = 4;
                }
                seqRobot++;
                break;
            }

            case 5: { // Strafe the robot to the left and off the shuttle

                // Define drive train target position and motor power.
                targetDrDistInch = 5f; // Set target distance
                targetPower = 0.05d;  // Set power

                // Use this OpModes's custom cmdMoveR method to calculate new target (in encoder
                // counts) and to initiate the move. cmdMoveR initiates a relative move.
                // cmdMove Parameters (distance inches, encoder count per inch, power, motor).
                targetPosLeftA = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower,
                        motorLeftA);
                targetPosLeftB = cmdMoveR(-targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower,
                        motorLeftB);
                targetPosRightA = cmdMoveR(-targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE,
                        -targetPower, motorRightA);
                targetPosRightB = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower,
                        motorRightB);

                seqRobot++;
                break;
            }

            case 15:
            case 9:
            case 6:
                // Hold until drive train move is complete
            {
                // Use this OpModes's custom chkMove to determine if motor move(s) are complete
                // chkMove Parameters (motor, target, allowed +/- error counts from target)
                // May need to add motor-is-busy check to ensure electric breaking complete.
                // May need to compensate for motor power if one motor is faster than another to
                // keep straight line.
                if (chkMove(motorLeftA, targetPosLeftA, ERROR_DRV_POS) &&
                        chkMove(motorLeftB, targetPosLeftB, ERROR_DRV_POS) &&
                        chkMove(motorRightA, targetPosRightA, ERROR_DRV_POS) &&
                        chkMove(motorRightB, targetPosRightB, ERROR_DRV_POS))
                {    // If drive train at target, hold position for X amount of time to stabilize
                    // motors.
                    seqRobot++;
                }
                break;
            }

            case 16:
            case 10:
            case 7:// Reset encoders
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

            case 8: { // Drive forward towards the depot

                // Define drive train target position and motor power.
                targetDrDistInch = 20; // Set target distance
                targetPower = 0.5d;  // Set power

                // Use this OpModes's custom cmdMoveR method to calculate new target (in encoder
                // counts) and to initiate the mov\e. cmdMoveR initiates a relative move.
                // cmdMove Parameters (distance inches, encoder count per inch, power, motor).
                targetPosLeftA = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower,
                        motorLeftA);
                targetPosLeftB = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower,
                        motorLeftB);
                targetPosRightA = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower,
                        motorRightA);
                targetPosRightB = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower,
                        motorRightB);

                seqRobot++;
                break;
            }

            case 11: { // Turn the robot to drop off the marker
                if (gyro() >= 44 || gyro() <= 46) {
                    motorLeftA.setPower(0);
                    motorLeftB.setPower(0);
                    motorRightA.setPower(0);
                    motorRightB.setPower(0);
                    seqRobot++;
                    timer.reset();
                } else if ( gyro() < 44){
                    motorLeftA.setPower(0.2);
                    motorLeftB.setPower(0.2);
                    motorRightA.setPower(-0.2);
                    motorRightB.setPower(-0.2);
                } else if (gyro() > 46) {
                    motorLeftA.setPower(-0.1);
                    motorLeftB.setPower(-0.1);
                    motorRightA.setPower(0.1);
                    motorRightB.setPower(0.1);
                }
                break;
            }

            case 12: { // Drop Marker
                servoDepotDrop.setPosition(DEPOT_DOWN);
                if (timer.milliseconds() > 3000) // Wait 3 seconds
                {
                    seqRobot++;
                    timer.reset();
                }
                break;
            }

            case 13: { // Retract marker deployment mechanism
                servoDepotDrop.setPosition(DEPOT_UP);
                if (timer.milliseconds() > 500) // Wait 0.5 second
                {
                    seqRobot++;
                }
                break;
            }

            case 14: { // Move away from the depot
                // Define drive train target position and motor power.
                targetDrDistInch = 7; // Set target distance
                targetPower = 0.5d;  // Set power

                // Use this OpModes's custom cmdMoveR method to calculate new target (in encoder
                // counts) and to initiate the mov\e. cmdMoveR initiates a relative move.
                // cmdMove Parameters (distance inches, encoder count per inch, power, motor).
                targetPosLeftA = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower,
                        motorLeftA);
                targetPosLeftB = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower,
                        motorLeftB);
                targetPosRightA = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower,
                        motorRightA);
                targetPosRightB = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower,
                        motorRightB);

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
