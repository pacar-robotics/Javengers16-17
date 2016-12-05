package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "LaunchGateCalibrationOp", group = "Test")

public class LauncherGateCalibration extends vv_OpMode {

    vv_Lib vvLib;


    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */


        vvLib = new vv_Lib(this);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Hello Driver", ":I am", ":ready!");    //
        telemetry.addData("Move Robot over line", ":To read", ":values!");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        vvLib.turnBeaconColorSensorLedOn(this);

        waitForStart();

        telemetry.setAutoClear(true);
        for (double i = 0.0f; i < 1.0f; i += 0.1f) {
            vvLib.setLauncherGatePosition(this, i);
            telemetryAddData("Servo Position", "Value;", "" + vvLib.getLauncherGatePosition(this));
            telemetryUpdate();
            Thread.sleep(2000);
        }
        idle();

    }

}


