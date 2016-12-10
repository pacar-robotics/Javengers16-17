package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.Backward;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.Forward;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysLeft;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysRight;


@TeleOp(name = "WheelCalibrationOp", group = "Calibrations")
public class WheelsCalibration extends vv_OpMode {

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

        telemetryAddData("Test", "Move", "Moving Forward");
        telemetryUpdate();

        vvLib.moveWheels(this, 12, 0.5f, Forward, true);

        Thread.sleep(3000);

        telemetryAddData("Test", "Move", "Moving Backward");
        telemetryUpdate();

        vvLib.moveWheels(this, 12, 0.5f, Backward, true);

        Thread.sleep(3000);

        telemetryAddData("Test", "Move", "Moving Sideways Left");
        telemetryUpdate();

        vvLib.moveWheels(this, 12, 0.5f, SidewaysLeft, true);

        Thread.sleep(3000);


        telemetryAddData("Test", "Move", "Moving Sideways Right");
        telemetryUpdate();

        vvLib.moveWheels(this, 12, 0.5f, SidewaysRight, true);

        Thread.sleep(3000);


    }

}