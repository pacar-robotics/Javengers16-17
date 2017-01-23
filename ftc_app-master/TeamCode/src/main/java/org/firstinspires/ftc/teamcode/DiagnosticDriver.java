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

        //run all the tests we can.

        vvDiagLib.runAllTests(this);
        //now walk through the tests and process them.

        for (int i = 0; i <= vvDiagLib.robotTestArray.length; i++) {
            //list all tests and results.
            if (vvDiagLib.robotTestArray[i].getTestValidity(this)) {
                //its a valid test
                if (vvDiagLib.robotTestArray[i].getTestResult(this)) {
                    //this test passed.
                    //lets print this message out.
                    telemetryAddData("Test Result:",
                            vvDiagLib.robotTestArray[i].getTestName(this),
                            ":" + "Test Passed");
                    telemetryUpdate();

                } else {
                    //this test failed.
                    //lets print out the message.
                    telemetryAddData("Test Result:",
                            vvDiagLib.robotTestArray[i].getTestName(this),
                            ":" + "Test Failed");
                    switch (vvDiagLib.robotTestArray[i].getTestSeverity(this)) {
                        case CRITICAL:
                            telemetryAddData("Test Result:",
                                    "This is a CRITICAL SEVERITY ERROR",
                                    "Error");
                            break;

                        case HIGH:
                            telemetryAddData("Test Result:",
                                    "This is a HIGH SEVERITY",
                                    "Error");
                            break;
                        case MEDIUM:
                            telemetryAddData("Test Result:",
                                    "This is a MEDIUM SEVERITY ERROR",
                                    "Error");
                            break;
                        case LOW:
                            break;
                        case INFO:
                            break;
                    }

                    telemetryAddData("Recommendation:", "try this:", vvDiagLib.robotTestArray[i].getTestRecommendation(this));
                    telemetryUpdate();

                }

            }
            //wait and display our errors if any
            Thread.sleep(2000);
        }


    }


}