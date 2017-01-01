package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "WheelsForwardGyroStableCalibrationOp", group = "Calibrations")
public class WheelsForwardGyroStabilizedCalibration extends vv_OpMode {

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

        telemetryAddData("Test", "Move", "Moving Forward 48 inches");
        telemetryUpdate();

        falseCondition falseCondition = new falseCondition();
        vvLib.universalGyroStabilizedMoveRobotByAxisVelocity(this, 0.0f, 0.8f, 2000, falseCondition);

    }

    public class falseCondition implements vv_OpMode.StopCondition {
        //can be used as an empty condition, so the robot keeps running in universal movement
        public boolean StopCondition(vv_OpMode aOpMode) throws InterruptedException {
            return (false);
        }
    }
}

