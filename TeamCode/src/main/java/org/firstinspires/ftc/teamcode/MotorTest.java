package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Encoder Test", group="Test")
@Disabled
public class MotorTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor testMotor = null;

    @Override
    public void runOpMode() {
        testMotor = hardwareMap.dcMotor.get("Left Drive Train");

        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)

        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();

        // run motor for 10 seconds
        testMotor.setPower(1.0);
        while (opModeIsActive() && (runtime.time() < 10)) {
            telemetry.addData("Encoder", "%4.1f:  %d counts", runtime.time(), testMotor.getCurrentPosition());
            telemetry.update();
        }

        testMotor.setPower(0.0);

        telemetry.addData("Encoder", "%5.0f Counts Per Second", (double) (testMotor.getCurrentPosition()) / runtime.time());
        telemetry.update();
        while (opModeIsActive()) ;

    }
}