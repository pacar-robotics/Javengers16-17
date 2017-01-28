package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "PreInitOp", group = "TeleOp")

public class PreInit extends vv_OpMode {

    vv_Lib vvLib;
    vv_TeleLib vvTeleLib;


    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        telemetry.setAutoClear(true);


        vvLib = new vv_Lib(this);
        vvTeleLib = new vv_TeleLib();


        // Send telemetry message to signify robot waiting;
        telemetry.addData("TeleOp Driver", ":I am", ":ready!");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        while (opModeIsActive()) {
            //process the gameopad control till we have the arm below target.

        }

    }

}
