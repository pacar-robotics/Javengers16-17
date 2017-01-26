package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "ChooChooCalibrationOp", group = "Calibrations")

public class ChooChooCalibration extends vv_OpMode {

    vv_Lib vvLib;


    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */


        vvLib = new vv_Lib(this);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Hello Driver", ":I am", ":ready!");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)


        waitForStart();


        while (opModeIsActive()) {

            vvLib.showChooChooPositionOnTelemetry(this, true);
            processLaunch();
            idle();

        }

    }

    private void processLaunch()
            throws InterruptedException {


        if (gamepad1.a) {
            vvLib.dropBall(this);
            vvLib.shootBall(this);
            vvLib.setupShot(this);
        }

        if (gamepad1.b) {
            vvLib.setupShot(this);
        }


    }

}