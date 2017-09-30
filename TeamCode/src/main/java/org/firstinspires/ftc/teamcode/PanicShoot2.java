package org.firstinspires.ftc.teamcode;

/**
 * Created by steve on 1/21/2017.
 */

public class PanicShoot2 extends PanicAbstractAutonomous {


    @Override
    public void runOpMode() throws InterruptedException {
        // determines if we are running beacon or park
        parkProgram = false;

        initRobot(); // Contains "init" stuff

        waitForStart();

        // step 0. shoot two balls at center vortex
        setFlywheelSpeed(0.40);
        sleep(750);
        int i = 0;
        while (i < 2 && opModeIsActive()) { // loop two times
            particleFlipper.setPosition(0.65);
            sleep(700);
            particleFlipper.setPosition(0.46);
            if (i == 0) {
                sleep(2000);
            }
            i++; // i = i + 1
        }
        stopFlywheel();

        requestOpModeStop();

    }

    }
