package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysRight;


/**
 * Created by Rachael_ on 10/23/2016.
 */


@Autonomous(name = "BlueRightBeaconNoDelayOp", group = "HorshamTest")
public class AutoOpBlueRightNoDelayWithBeacon extends vv_OpMode {
    vv_Lib vvLib;
    vv_Robot vvRobot;
    double readingsArray[];

    public void runOpMode() throws InterruptedException {
        telemetryAddData("Initializing, Please wait", "", "");
        telemetryUpdate();

        vvLib = new vv_Lib(this);


        telemetryAddData("<< BLUE ALLIANCE : RIGHT TILE >>Ready to go!", "", "");
        telemetryUpdate();
        //Turn the LED on the Color Sensor mounted on the floor of the Robot on
        vvLib.turnFloorLightSensorLedOn(this);


        waitForStart();


        basic_auto_strategy();


    }

    public void basic_auto_strategy() throws InterruptedException {

        //orient robot to shoot the first ball from right tile.

        vvLib.moveWheels(this, 4.0f, 0.5f, SidewaysRight, false);
        vvLib.turnAbsoluteMxpGyroDegrees(this, -15f);

        //first shoot two balls.
        vvLib.shootBallsAutonomousCommonAction(this);

        //move robot diagonally to corner in prep for scoring beacons

        vvLib.universalMoveRobot(this, 133, 0.99, 0.0, 2900,
                vvLib.rangeSensorUltraSonicCornerPositioningStop, false, 0, 0);

        vvLib.blueBeaconAutonomousCommonAction(this);

    }

}
