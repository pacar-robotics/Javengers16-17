package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "UniversalRobotSidewaysTestOp", group = "Calibrations")
public class UniversalRobotSidewaysTest extends vv_OpMode {

    /* Declare OpMode members. */
    vv_Lib vvLib;

    @Override
    public void runOpMode() throws InterruptedException {


        //Initialize library which in turn initializes the robot plus its hardware map
        //We need to pass the this pointer into vv_Lib in order to call some value added functions
        //in vv_Opmode

        telemetryAddData("Initializing", ":Please", "wait..");
        telemetryUpdate();
        DBG("before try");

        DBG("Before vvLIb init");
        vvLib = new vv_Lib(this);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver", "Im Ready");    //
        telemetry.update();
        DBG("before waitForStart");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetryAddData("Test", "Move", "Moving Sideways Right 2.5 sec");
        telemetryUpdate();

        vvLib.universalMoveRobotForDuration(this, 0.0, 0.5, 0.0, 2500);

        Thread.sleep(3000);


    }

}