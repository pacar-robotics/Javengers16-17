package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;
import java.io.PrintWriter;

/**
 * Using gamepad controls, the user can set the power factor for Teleop.
 * Writes power factor onto file in which the Teleop File can read.
 */

@TeleOp(name = "SetPowerFactorForTeleOp", group = "Calibration")
public class SetPowerFactor extends vv_OpMode {

    vv_Lib vvLib;

    float powerFactor = 0.5f;
    boolean powerFactorConfirmed = false;

    @Override
    public void runOpMode() throws InterruptedException{

        vvLib = new vv_Lib(this);
        //run init

        //tell driver the robot is ready
        telemetryAddData("Hello Driver: ", "I am ready", "");
        telemetryUpdate();

        //waits until driver presses start
        waitForStart();

        telemetryAddData("Passed WaitforStart: ", "Something Wrong with Update", "");

        while (opModeIsActive()) {


            if(!powerFactorConfirmed) {
            telemetryAddData("Change the Power Factor", " using up and down on DPAD", "");
            telemetryAddData(String.valueOf(powerFactor), " Press A to Confirm", "");
            telemetryUpdate();
            }


            if(gamepad1.dpad_down && powerFactor >= .2f) {
                powerFactor = powerFactor - .05f;
                Thread.sleep(250);
            }

            if(gamepad1.dpad_up && powerFactor <= 1.0) {
                powerFactor = powerFactor + .05f;
                Thread.sleep(250);
            }

            if (gamepad1.a) {
                telemetryAddData("Power Factor set to ", String.valueOf(powerFactor), "");
                telemetryUpdate();
                powerFactorConfirmed = true;
            }

        }
    }


}
