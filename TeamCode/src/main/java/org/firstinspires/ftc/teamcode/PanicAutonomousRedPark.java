package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "Panic: Red Park", group = "Panic: Red")
public class PanicAutonomousRedPark extends PanicAbstractAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        // determines if we are running beacon or park
        parkProgram = true;

        initRobot(); // Contains "init" stuff

        waitForStart();

        // wait 10 seconds
        //sleep(10000);

        // step 1. go forward
        shooterIsForward();
        runDriveTrainUntil(18); // runs until 18 inches has been run

        // step 2. spin CCW
        resetDriveTrainEncoders();
        spinCounterClockwise();
        motorRight.setTargetPosition(degreesToEncoderCount(43));
        while (motorRight.getCurrentPosition() < motorRight.getTargetPosition() && opModeIsActive()) {
            setDriveTrainPower(0.25);
        }
        stopDriveTrain();

        // step 3. shoot two balls at thingy
        startFlywheel();
        // this is the best speed for
        // Hitting the target
        sleep(850);
        int i = 0;
        while (i < 2 && opModeIsActive()) { // loop two times
//            if (i == 0) {
//                setFlywheelSpeed(flywheelSpeed + .05);
//            }
//            else
//            {
//                setFlywheelSpeed(flywheelSpeed);
//            }
            particleFlipper.setPosition(0.65);
            sleep(700);
            particleFlipper.setPosition(0.46);
            if(i == 0) {
                sleep(1300);
            }
            i++; // i = i + 1
        }
        stopFlywheel();

        // step 4. turn CCW again
        resetDriveTrainEncoders();
        spinCounterClockwise();
        motorRight.setTargetPosition(degreesToEncoderCount(28));
        while (motorRight.getCurrentPosition() < motorRight.getTargetPosition() && opModeIsActive()) {
            setDriveTrainPower(0.25);
        }
        stopDriveTrain();

        // step 5. go forward
        resetDriveTrainEncoders();
        shooterIsForward();
        runDriveTrainUntil(64);

        // step 6. turn CW
        resetDriveTrainEncoders();
        spinClockwise();
        motorLeft.setTargetPosition(degreesToEncoderCount(84));
        while (motorLeft.getCurrentPosition() < motorLeft.getTargetPosition() && opModeIsActive()) {
            setDriveTrainPower(0.25);
        }
        resetDriveTrainEncoders();

        // step 7. go forward until just past the ball
        shooterIsForward();
        runDriveTrainUntil(34);

        // step 8. turn CW
        resetDriveTrainEncoders();
        spinClockwise();
        motorLeft.setTargetPosition(degreesToEncoderCount(82));
        while (motorLeft.getCurrentPosition() < motorLeft.getTargetPosition() && opModeIsActive()) {
            setDriveTrainPower(0.25);
        }
        resetDriveTrainEncoders();

        // step 9. go forward enough to knock the ball off and park on the center vortes

        shooterIsForward();
        runDriveTrainUntil(19);

        // step 10.
        // We are done
    }
}
