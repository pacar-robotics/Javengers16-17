package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "EopdCalibrationOp", group = "Calibrations")
public class EopdCalibration extends vv_OpMode {

    /* Declare OpMode members. */
    vv_Lib vvLib;
    double readingsArray[];

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryAddData("Initializing", ":Please", "wait..");
        telemetryUpdate();
        DBG("before try");


        readingsArray = new double[9];

        //initialize array
        for (int i = 0; i < 9; i++) {
            readingsArray[i] = 0.0f;
        }

        //Initialize library which in turn initializes the robot plus its hardware map
        //We need to pass the this pointer into vv_Lib in order to call some value added functions
        //in vv_Opmode


        DBG("Before vvLIb init");
        vvLib = new vv_Lib(this);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver", "Im Ready");    //
        telemetry.update();
        DBG("before waitForStart");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        //looks like the EOPD is accurate to about  8cm inches.
        while (opModeIsActive()) {
            vvLib.showEopdValueOnTelemetry(this);
            idle();

        }

    }


}