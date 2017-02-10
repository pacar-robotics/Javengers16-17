package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "LineDetectCalibrationOp", group = "Calibrations")

public class FloorLightSensorCalibration extends vv_OpMode {

    vv_Lib vvLib;
    vv_TeleLib vvTeleLib;


    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */


        vvLib = new vv_Lib(this);
        vvTeleLib = new vv_TeleLib();


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Hello Driver", ":I am", ":ready!");    //
        telemetry.addData("Move Robot over line", ":To read", ":values!");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        vvLib.turnFloorColorSensorLedOn(this);

        waitForStart();


        while (opModeIsActive()) {

            vvLib.showFloorColorSensorIntensityOnTelemetry(this, true);
            telemetryUpdate();
            idle();

        }

    }

    public void processDrive()
            throws InterruptedException, vv_Robot.MotorNameNotKnownException {
        vvTeleLib.processFieldOrientedDrive(this, vvLib, 0.4f);
    }


}