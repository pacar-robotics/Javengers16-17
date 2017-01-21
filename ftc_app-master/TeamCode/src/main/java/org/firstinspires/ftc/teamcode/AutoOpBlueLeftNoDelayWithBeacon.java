package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * Created by Rachael_ on 10/23/2016.
 */


@Autonomous(name = "BlueLeftBeaconNoDelayOp", group = "HorshamTest")
public class AutoOpBlueLeftNoDelayWithBeacon extends vv_OpMode {
    vv_Lib vvLib;
    vv_Robot vvRobot;
    double readingsArray[];

    public void runOpMode() throws InterruptedException {
        telemetryAddData("Initializing, Please wait", "", "");
        telemetryUpdate();

        vvLib = new vv_Lib(this);


        telemetryAddData("<< BLUE ALLIANCE : LEFT TILE >> Ready to go!", "", "");
        telemetryUpdate();
        //Turn the LED on the Color Sensor mounted on the floor of the Robot on
        vvLib.turnFloorLightSensorLedOn(this);


        waitForStart();


        basic_auto_strategy();


    }

    public void basic_auto_strategy() throws InterruptedException {

        //first shoot two balls.
        vvLib.shootBallsAutonomousCommonAction(this);


        //move robot diagonally to corner in prep for scoring beacon
        vvLib.universalMoveRobot(this, 138, 0.99f, 0.0, 3700,
                vvLib.rangeSensorUltraSonicCornerPositioningStop, false, 0, 0);

        vvLib.blueAutonomousCommonAction(this);

    }

}
