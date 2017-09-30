package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by steve on 1/5/2017.
 */
@Autonomous(name = "TEST Vex Motor", group = "TEST")

public class PanicVexMotorTestAutonomous extends PanicAbstractAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        Servo test = hardwareMap.servo.get("TEST");

        waitForStart();

        while(opModeIsActive()) {

        if (opModeIsActive()) {
            long outTime = System.currentTimeMillis() + 5000;
            while (System.currentTimeMillis() < outTime && opModeIsActive()) {
                test.setDirection(Servo.Direction.FORWARD);
                test.setPosition(0.4);
            }
            test.setPosition(.5);
            sleep(500);
            outTime = System.currentTimeMillis() + 5000;
            while (System.currentTimeMillis() < outTime && opModeIsActive()) {
                test.setDirection(Servo.Direction.REVERSE);
                test.setPosition(0.4);
            }
            test.setPosition(.5);

        }
        }
    }
}

