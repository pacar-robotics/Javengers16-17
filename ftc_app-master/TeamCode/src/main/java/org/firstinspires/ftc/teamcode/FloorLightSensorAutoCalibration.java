package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;


@TeleOp(name = "LineDetectAutoCalibrationOp", group = "Calibrations")

public class FloorLightSensorAutoCalibration extends vv_OpMode {

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
        double minFloorLightCalibValue = 500; //color Sensor returns from 0 to 100 or 250;
        //this value is only meant to be higher than highest possible,
        // it can be any high number than max return
        //we know we get values in the 40s.
        double currentFloorLightCalibValue = 0;

        vvLib.moveForward(this, .2f);

        reset_timer();

        while (time_elapsed() <= 3000) {
            currentFloorLightCalibValue = vvLib.getFloorColorIntensity(this); //read value only once to avoid race or
            //side effects, because robot is moving at the same time, each call to getFloorColorIntensity in the ternary
            //operator below will return new values.
            //variables are cheap :)

            maxFloorLightCalibValue = (currentFloorLightCalibValue > maxFloorLightCalibValue ?
                    currentFloorLightCalibValue : maxFloorLightCalibValue);
            minFloorLightCalibValue = (currentFloorLightCalibValue < minFloorLightCalibValue ?
                    currentFloorLightCalibValue : minFloorLightCalibValue);

            if (time_elapsed() > 1500) {
                vvLib.moveBackward(this, .2f);
            }

            idle();
        }

        //we are done, stop all motors, so we dont have problems ending op.
        vvLib.stopAllMotors(this);

        float floorWhiteThreshold = (float) (maxFloorLightCalibValue + minFloorLightCalibValue) / 2;

        floorWhiteThresholdFileIO = new CalibFileIO(this, "floorWhiteThreshold");

        try {
            floorWhiteThresholdFileIO.writeTextFile(this, floorWhiteThreshold);
            telemetryAddData("Wrote File Successfully ", "Floor White Threshold: ", String.valueOf(floorWhiteThreshold));
            telemetryUpdate();

            didWrite = true;
        } catch (IOException e) {
            telemetryAddData("Problem: ", e.getMessage(), "");
            telemetryUpdate();
        }
        Thread.sleep(5000); //display any messages
    }
}
