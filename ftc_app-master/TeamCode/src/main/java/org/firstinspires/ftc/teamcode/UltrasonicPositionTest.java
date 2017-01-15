package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysRight;


@TeleOp(name = "UltrasonicPositionTest", group = "Calibrations")
public class UltrasonicPositionTest extends vv_OpMode {

    /* Declare OpMode members. */
    vv_Lib vvLib;
    double readingsArray[];

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryAddData("Initializing", ":Please", "wait..");
        telemetryUpdate();
        DBG("before try");


        //Initialize library which in turn initializes the robot plus its hardware map
        //We need to pass the this pointer into vv_Lib in order to call some value added functions
        //in vv_Opmode


        DBG("Before vvLIb init");
        vvLib = new vv_Lib(this);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver", "Im Ready");    //
        telemetry.update();
        DBG("before waitForStart");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        telemetryAddData("Positioning Robot to be 4 inches distance from wall", ":", ".");
        telemetryUpdate();


        double distanceToWall = vvLib.getUltrasonicDistance(this); //in inches
        vvLib.moveWheels(this, (float) (distanceToWall - 4.0), 0.6f, SidewaysRight, true);


    }

}