package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;


@TeleOp(name = "AutoLineDetectCalibrationOp", group = "Calibrations")

public class AutoFloorLightSensorCalibration extends vv_OpMode {

    vv_Lib vvLib;
    CalibFileIO floorWhiteThresholdFileIO;
    boolean didWrite = false;

    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */


        vvLib = new vv_Lib(this);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Hello Driver", ":I am", ":ready!");    //
        telemetry.addData("Move Robot near line", ":To read", ":values!");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        vvLib.turnFloorColorSensorLedOn(this);


        waitForStart();

        double maxFloorLightCalibValue = 0;
        double minFloorLightCalibValue = 1;

        vvLib.moveForward(this, .3f);

        reset_timer();

        while (time_elapsed() <= 6000) {
            maxFloorLightCalibValue = (vvLib.getFloorColorIntensity(this) > maxFloorLightCalibValue ? vvLib.getFloorColorIntensity(this) : maxFloorLightCalibValue);
            minFloorLightCalibValue = (vvLib.getFloorColorIntensity(this) > minFloorLightCalibValue ? vvLib.getFloorColorIntensity(this) : minFloorLightCalibValue);

            if (time_elapsed() > 3000) {
                vvLib.moveBackward(this, .3f);
            }

            idle();
        }

        float floorWhiteThreshold = (float) (maxFloorLightCalibValue + minFloorLightCalibValue) / 2;

        floorWhiteThresholdFileIO = new CalibFileIO("floorWhiteThreshold");

        try {
            floorWhiteThresholdFileIO.writeTextFile(floorWhiteThreshold);
            telemetryAddData("Wrote File Successfully ", "Floor White Threshold: ", String.valueOf(floorWhiteThreshold));
            telemetryUpdate();
            didWrite = true;
        } catch (IOException e) {
            telemetryAddData("Problem: ", e.getMessage(), "");
            telemetryUpdate();
        }

    }
}
