package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "CapBallCalibrationOp", group = "Calibrations")

public class CapBallCalibration extends vv_OpMode {

    vv_Lib vvLib;


    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        telemetry.setAutoClear(true);


        vvLib = new vv_Lib(this);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Hello Driver", ":I am", ":ready!");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        while (opModeIsActive()) {
            try {
                vvLib.controlCapBallWithoutLimits(this);
                telemetry.addData("Cap Ball Encoder:", "Value:",
                        "Is:" + vvLib.getCapBallPosition(this));
            } catch (vv_Robot.MotorStalledException MSE) {
                telemetryAddData("Motor Stalled!", "Name", MSE.getMessage());
                telemetryUpdate();
                Thread.sleep(500);
            }
            idle();
        }
    }

}