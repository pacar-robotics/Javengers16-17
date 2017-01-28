package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "CapBallReleaseServoCalibrationOp", group = "Calibrations")

public class CapBallReleaseServoPositionCalibration extends vv_OpMode {

    vv_Lib vvLib;
    vv_TeleLib vvTeleLib;


    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */


        vvLib = new vv_Lib(this);
        vvTeleLib = new vv_TeleLib();


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Hello Driver", ":I am", ":ready!");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        telemetry.setAutoClear(true);
        for (double i = 0.0f; i < 1.0f; i += 0.1f) {
            vvTeleLib.setCapBallReleaseServoPosition(this, vvLib, i);
            telemetryAddData("Cap Ball Servo Position", "Value;", "" +
                    vvTeleLib.getCapBallReleaseServoPosition(this, vvLib));
            telemetryUpdate();
            Thread.sleep(2000);
            idle();
        }

    }

}


