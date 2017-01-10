package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysLeft;
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
        vvLib.turnFloorLightSensorLedOn(this);


        waitForStart();


        basic_auto_strategy();


    }

    public void basic_auto_strategy() throws InterruptedException {


        //lets move the robot at approx 60 degrees till we detect the line.


        //move the robot with no pulsed movement for a max duration of 3.5 seconds.
        //we want to detect the line orthogonally not at a smaller variable angle for
        //consistency in positioning.
        //duration is found by estimation and test.


        //first shoot two balls.

        //** disabled to allow for mechanical fixes to hold the choo - choo to frame
        //12/31/2016

        vvLib.moveWheels(this, 4, .4f, SidewaysRight, true);
        vvLib.turnAbsoluteGyroDegrees(this, -20.0f);

        // Shoot the first ball
        vvLib.shootBall(this);
        vvLib.setupShot(this);

        //drop ball
        vvLib.dropBall(this);

        //Shoot the second ball.
        vvLib.shootBall(this);
        vvLib.shootBall(this);

        vvLib.turnAbsoluteGyroDegrees(this, 0);

        //move robot diagonally in prep for first beacon

        vvLib.universalMoveRobotByAxisVelocity(this, 0.60, 0.45, 0.0, 3200, vvLib.falseStop, false, 0, 0);

        //rotate to face beacon
        vvLib.turnAbsoluteMxpGyroDegrees(this, -90); //with trim

        //detect the line and score beacon.

        vvLib.ScoreBeaconFromTheLeft(this);

        //now to work on second beacon.


        //now move to second beacon

        //pull back 6 inches to create clearance
        vvLib.moveWheels(this, 6, 0.8f, SidewaysLeft, true);

        //orient to 90 degrees to field

        Thread.sleep(50);

        vvLib.turnAbsoluteMxpGyroDegrees(this, -90); //with trim


        //lets move over the first beacon line, to prevent stopping at wrong line.


        vvLib.universalMoveRobotByAxisVelocity(this, -0.4, -0.8, 0.0, 2600, vvLib.falseStop, false, 0, 0);

        vvLib.turnAbsoluteMxpGyroDegrees(this, -90); //with trim


        vvLib.ScoreBeaconFromTheLeft(this);

        vvLib.moveWheels(this, 10, .8f, SidewaysLeft, true);
        Thread.sleep(25);
        vvLib.turnAbsoluteMxpGyroDegrees(this, -135);
        Thread.sleep(25);

        vvLib.universalMoveRobotByAxisVelocity(this, 0.0f, 9.0f, 0.0, 2000, vvLib.falseStop, false, 0, 0);


    }

}
