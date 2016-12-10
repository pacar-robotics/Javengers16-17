package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "BeaconColorCalibrationOp", group = "Calibrations")

public class BeaconColorCalibration extends vv_OpMode {

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

        vvLib.turnBeaconColorSensorLedOff(this);

        waitForStart();


        while (opModeIsActive()) {
            telemetry.setAutoClear(true);
            vvLib.showBeaconColorValuesOnTelemetry(this, true);
            processBeacon();
            idle();
        }

    }


    private void processBeacon() throws InterruptedException {
        // Changes Beacon Mechanism to left position in order to score the beacon
        if (gamepad1.dpad_left) {
            vvLib.pressLeftBeaconButton(this);
        }
        // Changes Beacon Mechanism to right position in order to score the beacon
        if (gamepad1.dpad_right) {
            vvLib.pressRightBeaconButton(this);
        }

    }


}