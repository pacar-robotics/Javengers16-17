package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "BeaconTestOp", group = "Calibrations")

public class BeaconTest extends vv_OpMode {

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

        telemetry.setAutoClear(true);

        telemetryAddData("Testing for TEAM BLUE:", "Value;", ":Reposition to touch Beacon");
        telemetryUpdate();
        Thread.sleep(10000);

        telemetryAddData("TEAM BLUE:", "Value;", ":Looking for color");
        telemetryUpdate();
        Thread.sleep(2000);

        if (vvLib.getBeaconLeftColor(this) == vv_Constants.BeaconColorEnum.BLUE) {
            telemetryAddData("TEAM BLUE:", "Value;", ":Found Blue");
            telemetryUpdate();
            Thread.sleep(2000);
            //must press right button
            telemetryAddData("TEAM BLUE:", "Value;", ":Pressing LEFT Button");
            telemetryUpdate();
            Thread.sleep(2000);
            vvLib.pressLeftBeaconButton(this);
            Thread.sleep(1000);
        }

        if (vvLib.getBeaconLeftColor(this) == vv_Constants.BeaconColorEnum.RED) {
            telemetryAddData("TEAM BLUE:", "Value;", ":Found RED");
            telemetryUpdate();
            Thread.sleep(2000);
            //must press right button
            telemetryAddData("TEAM BLUE:", "Value;", ":Pressing RIGHT Button");
            telemetryUpdate();
            Thread.sleep(2000);
            vvLib.pressRightBeaconButton(this);
            Thread.sleep(1000);
        }

        if (vvLib.getBeaconLeftColor(this) == vv_Constants.BeaconColorEnum.UNKNOWN) {
            telemetryAddData("TEAM BLUE:", "Value;", ":Found NO COLOR");
            telemetryUpdate();
            Thread.sleep(2000);
        }

        telemetryAddData("Testing for TEAM RED:", "Value;", ":Reposition to touch Beacon");
        telemetryUpdate();
        Thread.sleep(10000);


        telemetryAddData("TEAM RED:", "Value;", ":Looking for color");
        telemetryUpdate();
        Thread.sleep(2000);

        if (vvLib.getBeaconLeftColor(this) == vv_Constants.BeaconColorEnum.BLUE) {
            telemetryAddData("TEAM RED:", "Value;", ":Found Blue");
            telemetryUpdate();
            Thread.sleep(2000);
            //must press right button
            telemetryAddData("TEAM RED:", "Value;", ":Pressing RIGHT Button");
            telemetryUpdate();
            Thread.sleep(2000);
            vvLib.pressRightBeaconButton(this);
            Thread.sleep(1000);
        }

        if (vvLib.getBeaconLeftColor(this) == vv_Constants.BeaconColorEnum.RED) {
            telemetryAddData("TEAM RED:", "Value;", ":Found RED");
            telemetryUpdate();
            Thread.sleep(2000);
            //must press right button
            telemetryAddData("TEAM RED:", "Value;", ":Pressing LEFT Button");
            telemetryUpdate();
            Thread.sleep(2000);
            vvLib.pressLeftBeaconButton(this);
            Thread.sleep(1000);
        }

        if (vvLib.getBeaconLeftColor(this) == vv_Constants.BeaconColorEnum.UNKNOWN) {
            telemetryAddData("TEAM RED:", "Value;", ":Found NO COLOR");
            telemetryUpdate();
            Thread.sleep(2000);
        }


    }

}


