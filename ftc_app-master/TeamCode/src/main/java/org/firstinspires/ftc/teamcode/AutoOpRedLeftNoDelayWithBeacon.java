package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysRight;


/**
 * Created by Rachael_ on 10/23/2016.
 */


@Autonomous(name = "RedLeftBeaconNoDelayOp", group = "HorshamTest")
public class AutoOpRedLeftNoDelayWithBeacon extends vv_OpMode {
    vv_Lib vvLib;
    vv_Robot vvRobot;
    double readingsArray[];

    public void runOpMode() throws InterruptedException {
        telemetryAddData("Initializing, Please wait", "", "");
        telemetryUpdate();

        vvLib = new vv_Lib(this);


        telemetryAddData("<< RED ALLIANCE >>:LEFT TILE  Ready to go!", "", "");
        telemetryUpdate();
        //Turn the LED on the Color Sensor mounted on the floor of the Robot on
        vvLib.turnFloorColorSensorLedOn(this);


        waitForStart();


        basic_auto_strategy();


    }

    public void basic_auto_strategy() throws InterruptedException {

        //first orient to shoot the balls

        vvLib.moveWheels(this, 4, .4f, SidewaysRight, true);
        vvLib.turnAbsoluteMxpGyroDegrees(this, -5.0f);

        //shoot the 2 balls.
        vvLib.shootBallsAutonomousCommonAction(this);


        //rotate robot 90 degrees so the range sensor is facing beacon side wall
        //this enables the ultrasonic range sensors being correctly set.

        vvLib.turnAbsoluteMxpGyroDegrees(this, -90);
        //move robot diagonally in prep for first beacon
        vvLib.universalMoveRobot(this, 140, 0.45, 0.0, 2900,
                vvLib.rangeSensorUltraSonicCornerPositioningStop, false, 0, 0);

        //perform actions common to both red positions

        vvLib.redBeaconAutonomousCommonAction(this);


    }

}
