package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;

/**
 * Using gamepad controls, the user can set the power factor for Teleop.
 * Writes power factor onto file in which the Teleop File can read.
 */

@TeleOp(name = "SetPowerFactorForTeleOp", group = "Calibration")
public class SetPowerFactor extends vv_OpMode {

    vv_Lib vvLib;
    LightCalibFileIO powerFactorFileIO;

    float drivePowerFactor = 0.5f;
    boolean powerFactorConfirmed = false;
    boolean didWrite = false;

    @Override
    public void runOpMode() throws InterruptedException{

        vvLib = new vv_Lib(this);
        //run init
        powerFactorFileIO = new LightCalibFileIO();

        //tell driver the robot is ready
        telemetryAddData("Hello Driver: ", "I am ready", "");
        telemetryUpdate();

        //waits until driver presses start
        waitForStart();

        telemetryAddData("Passed WaitforStart: ", "Something Wrong with Update", "");

        while (opModeIsActive()) {


            while (!powerFactorConfirmed) {
                telemetryAddData("Change the Power Factor", " using up and down on DPAD", "");
                telemetryAddData(String.valueOf(drivePowerFactor), " Press A to Confirm", "");
                telemetryUpdate();


                if (gamepad1.dpad_down && drivePowerFactor >= .2f) {
                    drivePowerFactor = drivePowerFactor - .05f;
                    Thread.sleep(250);
                }

                if (gamepad1.dpad_up && drivePowerFactor <= 1.0) {
                    drivePowerFactor = drivePowerFactor + .05f;
                    Thread.sleep(250);
                }

                if (gamepad1.a) {
                    powerFactorConfirmed = true;
                }
            }

//            if(vvLib.createFile(this, "PowerFactor"))
//            {
//                telemetryAddData("Wrote File Successfully: ", "PowerFactor", "");
//                telemetryUpdate();
//                Thread.sleep(1000);
//            } else {
//                telemetryAddData("Something went wrong in creating File", "", "");
//                Thread.sleep(1000);
//            }
                try {
                    powerFactorFileIO.writeTextFile(drivePowerFactor);
                    telemetryAddData("Wrote File Successfully ", "Power Factor: ", String.valueOf(drivePowerFactor));
                    telemetryUpdate();
                    didWrite = true;
                } catch (IOException e) {
                    telemetryAddData("Problem: ", e.getMessage(), "");
                    telemetryUpdate();
                }

//            Thread.sleep(2000);
//
//            try {
//                drivePowerFactor = powerFactorFileIO.getCalibrationValue();
//                telemetryAddData("Power Factor: ", String.valueOf(drivePowerFactor), "");
//                telemetryUpdate();
//            } catch (IOException e) {
//                telemetryAddData("Problem: ", e.getMessage(), "");
//                drivePowerFactor = vv_Constants.STANDARD_DRIVE_POWER_FACTOR;
//                telemetryAddData("Power Factor: ", String.valueOf(drivePowerFactor), "");
//                telemetryUpdate();
//            }

        }
    }
}
