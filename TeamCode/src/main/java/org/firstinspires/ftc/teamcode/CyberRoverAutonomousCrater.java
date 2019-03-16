package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RoverCrater", group = "RiderModes")
public class CyberRoverAutonomousCrater extends CyberRoverAbstract{

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

            case 2: { // Strafe the robot to the left and off the shuttle

                // Define drive train target position and motor power.E
                targetDrDistInch = -55f; // Set target distance
                targetPower = -0.5d;  // Set power

                // Use this OpModes's custom cmdMoveR method to calculate new target (in encoder
                // counts) and to initiate the move. cmdMoveR [-;initiates a relative move.
                // cmdMove Parameters (distance inches, encoder count per inch, power, motor).
                targetPosLeftA = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower,
                        motorLeftA);
                targetPosLeftB = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, -targetPower,
                        motorLeftB);
                targetPosRightA = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, -targetPower,
                        motorRightA);
                targetPosRightB = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower,
                        motorRightB);

                seqRobot++;
                break;
            }

            case 3:
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

            case 4:// Reset encoders
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

} // End OpMode