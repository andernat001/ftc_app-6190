package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name = "Panic: Blue Beacon", group = "Panic: Blue")
public class PanicAutonomousBlueBeacon extends PanicAbstractAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        // determines if we are running beacon or park
        parkProgram = false;

        initRobot(); // Contains "init" stuff

        waitForStart();

        // step 0. shoot two balls at center vortex
        startFlywheel();
        sleep(900);
        int i = 0;
        while (i < 2 && opModeIsActive()) { // loop two times
//            if (i == 0) {
//                setFlywheelSpeed(flywheelSpeed - 0.08);
//            }
//            else
//            {
//                setFlywheelSpeed(flywheelSpeed - 0.08);
//            }
            particleFlipper.setPosition(0.65);
            sleep(700);
            particleFlipper.setPosition(0.46);
            if (i == 0) {
                sleep(250);
                cycleBall();
                sleep(500);
            }
            i++; // i = i + 1
        }
        stopFlywheel();

        // step 1. go forward
        shooterIsForward();
        resetDriveTrainEncoders();
        runDriveTrainUntil(31);

        // step 2. spin CW
        spinClockwise();
        resetDriveTrainEncoders();
        motorLeft.setTargetPosition(degreesToEncoderCount(85));
        while (motorLeft.getCurrentPosition() < motorLeft.getTargetPosition() && opModeIsActive()) {
            setDriveTrainPower(0.25);
        }
        stopDriveTrain();

        // step 2.5 go forward
        shooterIsForward();
        resetDriveTrainEncoders();
        runDriveTrainUntil(34);

        // step 2.6 spin cw
        spinClockwise();
        resetDriveTrainEncoders();
        motorLeft.setTargetPosition(degreesToEncoderCount(87));
        while (motorLeft.getCurrentPosition() < motorLeft.getTargetPosition() && opModeIsActive()) {
            setDriveTrainPower(0.25);
        }

        // step 3. go forward until we hit the white line
        buttonPusherIsForward();
        resetDriveTrainEncoders();
        findWhiteLine(16);

        // step 4. spin CW
        resetDriveTrainEncoders();
        spinClockwise();
        motorLeft.setTargetPosition(degreesToEncoderCount(87));
        while (motorLeft.getCurrentPosition() < motorLeft.getTargetPosition() && opModeIsActive()) {

            setDriveTrainPower(0.15);
        }
        stopDriveTrain();

        // step 5. go forward until Range Sensor says we are close enough
        //approachBeacon include the stop
        approachBeacon(7);

        // step 6. figure out if the beacon is red or blue and press red button
//        sleep(400);
        pressBlueButton();

        // step 7. back up
        resetDriveTrainEncoders();
        shooterIsForward();
        runDriveTrainUntil(6);

        // step 8. spin ccw
        spinCounterClockwise();
        resetDriveTrainEncoders();
        motorRight.setTargetPosition(degreesToEncoderCount(84));
        while (motorRight.getCurrentPosition() < motorRight.getTargetPosition() && opModeIsActive()) {
            setDriveTrainPower(0.25);
        }
        stopDriveTrain();

        //step 8.5 start journey to the white line
        buttonPusherIsForward();
        resetDriveTrainEncoders();
        runDriveTrainUntil(27);

        // step 9. go forward to white line
        // findWhiteLine include the stopping
        buttonPusherIsForward();
        resetDriveTrainEncoders();
        findWhiteLine(30);

        if (failSafe) {
            requestOpModeStop();
        }

        // step 10. spin until white line cw
        spinClockwise();
        resetDriveTrainEncoders();
        motorLeft.setTargetPosition(degreesToEncoderCount(89));
        while (motorLeft.getCurrentPosition() < motorLeft.getTargetPosition() && opModeIsActive()) {
            setDriveTrainPower(0.15);
        }
        stopDriveTrain();
        resetDriveTrainEncoders();

        // step 11. go forward until Range Sensor says we are 3" away
        //approachBeacon include the stop
        approachBeacon(7);

        // step 12. figure out if the beacon is red or blue and press red button
//        sleep(400);
        pressBlueButton();

        // step 13. attempt to get to the center vortex
        resetDriveTrainEncoders();
        shooterIsForward();
        getBlueBall(54);

        // step 14.
        // We are done
    }

    private void getBlueBall(int distance)
    {
        if (opModeIsActive())
        {
            double targetInEncoderCounts = inchesToEncoderCount(distance);
            if (motorLeft.getDirection().equals(DcMotorSimple.Direction.FORWARD))
            {
                while (motorLeft.getCurrentPosition() < targetInEncoderCounts && opModeIsActive()) {
                    telemetry.addData("Left Encoder: ", motorLeft.getCurrentPosition());
                    telemetry.update();
                    motorLeft.setPower(.47);
                    motorRight.setPower(1.0);
                }
            }
            stopDriveTrain();
        }
    }
}
