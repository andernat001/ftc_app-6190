package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name = "Panic: Red Beacon", group = "Panic: Red")
public class PanicAutonomousRedBeacon extends PanicAbstractAutonomous {

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
//                setFlywheelSpeed(flywheelSpeed);
//            }
//            else
//            {
//                setFlywheelSpeed(flywheelSpeed);
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
        runDriveTrainUntil(30);

        // step 2. spin CCW
        spinCounterClockwise();
        resetDriveTrainEncoders();
        motorRight.setTargetPosition(degreesToEncoderCount(86));
        while (motorRight.getCurrentPosition() < motorRight.getTargetPosition() && opModeIsActive()) {
            setDriveTrainPower(0.25);
        }
        stopDriveTrain();

        // step 2.5 go forward
        shooterIsForward();
        resetDriveTrainEncoders();
        runDriveTrainUntil(43);

        // step 2.6 spin ccw
        spinCounterClockwise();
        resetDriveTrainEncoders();
        motorRight.setTargetPosition(degreesToEncoderCount(86));
        while (motorRight.getCurrentPosition() < motorRight.getTargetPosition() && opModeIsActive()) {
            setDriveTrainPower(0.25);
        }

        // step 3. go forward until we hit the white line
        buttonPusherIsForward();
        resetDriveTrainEncoders();
        findWhiteLine(16);

        // step 3.5 go backward 0.5 inch after we hit the white line
        shooterIsForward();
        resetDriveTrainEncoders();
        runDriveTrainUntil(0.5);

        // step 4. spin CCW
        buttonPusherIsForward();
        resetDriveTrainEncoders();
        spinCounterClockwise();
        motorRight.setTargetPosition(degreesToEncoderCount(89));
        while (motorRight.getCurrentPosition() < motorRight.getTargetPosition() && opModeIsActive()) {

            setDriveTrainPower(0.15);
        }
        stopDriveTrain();

        // step 5. go forward until Range Sensor says we are close enough
        //approachBeacon include the stop
        approachBeacon(7);

        // step 6. figure out if the beacon is red or blue and press red button
//        sleep(400);
        pressRedButton();

        // step 7. back up
        resetDriveTrainEncoders();
        shooterIsForward();
        runDriveTrainUntil(5.0);

        // step 8. spin cw
        spinClockwise();
        resetDriveTrainEncoders();
        motorLeft.setTargetPosition(degreesToEncoderCount(88));
        while (motorLeft.getCurrentPosition() < motorLeft.getTargetPosition() && opModeIsActive()) {
            setDriveTrainPower(0.25);
        }
        stopDriveTrain();

        // step 8.5 go forward 27 inches to begin journey to white line
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

        // step 9.5 go backward 0.5 inch after we hit the white line
        shooterIsForward();
        resetDriveTrainEncoders();
        runDriveTrainUntil(0.5);

        // step 10. spin until white line ccw
        spinCounterClockwise();
        resetDriveTrainEncoders();
        motorRight.setTargetPosition(degreesToEncoderCount(88));
        while (motorRight.getCurrentPosition() < motorRight.getTargetPosition() && opModeIsActive()) {
            setDriveTrainPower(0.15);
        }
        stopDriveTrain();
        resetDriveTrainEncoders();

        // step 11. go forward until Range Sensor says we are 3" away
        //approachBeacon include the stop
        approachBeacon(7);

        // step 12. figure out if the beacon is red or blue and press red button
//        sleep(400);
        pressRedButton();

        // step 13. attempt to get to the center vortex
        resetDriveTrainEncoders();
        shooterIsForward();
        getRedBall(68);

        // step 14.
        // We are done
    }

    private void getRedBall(int distance)
    {
        if (opModeIsActive())
        {
            double targetInEncoderCounts = inchesToEncoderCount(distance);
            if (motorLeft.getDirection().equals(DcMotorSimple.Direction.FORWARD))
            {
                while (motorLeft.getCurrentPosition() < targetInEncoderCounts && opModeIsActive()) {
                    telemetry.addData("Left Encoder: ", motorLeft.getCurrentPosition());
                    telemetry.update();
                    motorLeft.setPower(1.0);
                    motorRight.setPower(.51);
                }
            }
            stopDriveTrain();
        }
    }
}
