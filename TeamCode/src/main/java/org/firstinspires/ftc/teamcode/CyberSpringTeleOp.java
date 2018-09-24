package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Steve on 1/29/2018.
 */
@TeleOp(name = "Spring Training")
public class CyberSpringTeleOp extends CyberSpringAbstract {
    public CyberSpringTeleOp() {
    }

    @Override
    public void init() {
    }

    @Override
    public void loop() {

        motorRight.setPower(powerRight);
        motorLeft.setPower(powerLeft);

        velocityDrive = -gamepad1.left_stick_y;
        rotationDrive = -gamepad1.left_stick_x;

        powerRight = velocityDrive + rotationDrive;
        powerLeft = velocityDrive - rotationDrive;

        powerRight = (float) scaleInput(powerRight);
        powerLeft = (float) scaleInput(powerLeft);

        powerRight = Range.clip(powerRight, -1, 1);
        powerLeft = Range.clip(powerLeft, -1, 1);

        if (gamepad1.left_stick_x <= 0.05 && gamepad1.left_stick_x >= -0.05) {
            gamepad1.left_stick_x = 0;
        }
        if (gamepad1.left_stick_y <= 0.05 && gamepad1.left_stick_y >= -0.05) {
            gamepad1.left_stick_y = 0;
        }
    }
}