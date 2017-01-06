package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysLeft;
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


        //lets move the robot at approx 60 degrees till we detect the line.


        //move the robot with no pulsed movement for a max duration of 3.5 seconds.
        //we want to detect the line orthogonally not at a smaller variable angle for
        //consistency in positioning.
        //duration is found by estimation and test.


        //first shoot two balls.


        //orient robot to shoot the first ball.

        vvLib.moveWheels(this, 4.0f, 0.5f, SidewaysRight, false);
        vvLib.turnAbsoluteMxpGyroDegrees(this, -25f);

        // Shoot the first ball
        vvLib.shootBall(this);
        vvLib.setupShot(this);

        //drop ball
        vvLib.dropBall(this);

        //Shoot the second ball.
        vvLib.shootBall(this);
        vvLib.shootBall(this);


        //rotate back for reference before proceeding


        vvLib.turnAbsoluteMxpGyroDegrees(this, 0f);


        //move robot diagonally in prep for first beacon

        vvLib.universalMoveRobotByAxisVelocity(this, 0.85, -0.85, 0.0, 2800, vvLib.falseStop, false, 0, 0);

        //rotate to face beacon
        vvLib.turnAbsoluteMxpGyroDegrees(this, 90); //with trim

        //detect the line and score beacon.

        vvLib.ScoreBeaconFromTheRight(this);

        //now to work on second beacon.


        //now move to second beacon

        //pull back 4 inches to create clearance
        vvLib.moveWheels(this, 4, 0.8f, SidewaysLeft, true);

        //orient to 90 degrees to field


        vvLib.turnAbsoluteMxpGyroDegrees(this, 90); //with trim


        //lets move over the first beacon line, to prevent stopping at wrong line.

        //vvLib.moveWheels(this, 40.0f, 0.9f, Forward, true);

        vvLib.universalMoveRobotByAxisVelocity(this, -0.3, 0.8, 0.0, 1600, vvLib.falseStop, false, 0, 0);

        vvLib.turnAbsoluteMxpGyroDegrees(this, 90); //with trim


        vvLib.ScoreBeaconFromTheRight(this);

        vvLib.moveWheels(this, 10, .8f, SidewaysLeft, true);
        vvLib.turnAbsoluteMxpGyroDegrees(this, 140);
        Thread.sleep(10);

        vvLib.universalMoveRobotByAxisVelocity(this, 0.0, -0.9, 0.0, 2000, vvLib.falseStop, false, 0, 0);


    }

}
