package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.Backward;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.Forward;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysLeft;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysRight;


@TeleOp(name = "WheelCycleCalibrationOp", group = "Calibrations")
public class WheelsCycleCalibration extends vv_OpMode {

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

        telemetryAddData("Testig Wheel Cycling", "Cycle:", "Robot should be in same position " +
                "after each cycle");
        for (int i = 0; i < 10; i++) {

            telemetryAddData("Testing Wheel Cycling", "Cycle:", "Cycle:" + i);
            telemetryUpdate();

            vvLib.moveWheels(this, 4, 0.5f, Forward, true);

            Thread.sleep(500);

            vvLib.moveWheels(this, 4, 0.5f, Backward, true);

            Thread.sleep(500);


            vvLib.moveWheels(this, 4, 0.5f, SidewaysLeft, true);

            Thread.sleep(500);

            vvLib.moveWheels(this, 4, 0.5f, SidewaysRight, true);

            Thread.sleep(500);
        }


    }

}