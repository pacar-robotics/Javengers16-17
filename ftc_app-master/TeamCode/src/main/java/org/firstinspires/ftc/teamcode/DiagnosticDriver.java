package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "DiagnosticsDriverOp", group = "Diagnostics")

public class DiagnosticDriver extends vv_OpMode {

    vv_DiagLib vvDiagLib;


    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        telemetry.setAutoClear(true);

        vvDiagLib = new vv_DiagLib(this);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Hello Driver", ":I am", ":ready!");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        //process diagnostic commands here

        //run all the automatic tests we can.

        vvDiagLib.runAllAutomaticTests(this);
        //now walk through the tests and process them.
        vvDiagLib.analyzeTestResults(this);
        vvDiagLib.writeAllResults(this);
        Thread.sleep(5000);

    }


}