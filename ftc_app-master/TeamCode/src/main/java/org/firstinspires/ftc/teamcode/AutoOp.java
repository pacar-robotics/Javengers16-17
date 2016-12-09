package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.vv_Constants.BeaconServoStateEnum;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.Backward;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysLeft;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysRight;
import static org.firstinspires.ftc.teamcode.vv_Constants.TEAM_RED;

/**
 * Created by Rachael_ on 10/23/2016.
 */


@Autonomous(name = "BasicBlueRightOp", group = "Test")
public class AutoOp extends vv_OpMode
{
    vv_Lib vvLib;
    vv_Robot vvRobot;

    public void runOpMode() throws InterruptedException
    {
        telemetryAddData("Initializing, Please wait", "", "");
        telemetryUpdate();
        try {
            vvLib = new vv_Lib(this);
            telemetryAddData("Ready to go!", "", "");
            telemetryUpdate();
            //Turn the LED on the Color Sensor mounted on the floor of the Robot on
            vvLib.turnFloorLightSensorLedOn(this);


            waitForStart();


            basic_auto_strategy();



            //  vvLib.moveTillColor(this, vvRobot.getColorSensor(this));
        } catch (vv_Robot.MotorNameNotKnownException MNNKE) {
            telemetryAddData("Motor Not found", "Values:", MNNKE.getMessage());
            telemetryUpdate();
            Thread.sleep(3000);
        }

    }

    public void basic_auto_strategy() throws InterruptedException, vv_Robot.MotorNameNotKnownException {



        //test the turn of the robot.

        //clockwise is positive

        //power of 0.5 is recommended to prevent single motor stalls during turns
        //otherwise code has to be written to solve for this (maybe increase margin ?)

        vvLib.moveWheels(this, 4, .4f, SidewaysRight);

        vvLib.shootBall(this);
        vvLib.setupShot(this);

        vvLib.toggleIntake(this);
        Thread.sleep(750);
        vvLib.toggleIntake(this);
        vvLib.dropBall(this);
        vvLib.shootBall(this);
        vvLib.setupShot(this);

        Thread.sleep(250);

        vvLib.moveWheels(this, 8, 0.4f, SidewaysRight);

        Thread.sleep(250);

        vvLib.turnAbsoluteGyroDegrees(this, 55);

        Thread.sleep(250);

        //distances are by experimentation.

        // vvLib.moveWheels(this, 60, 0.9f, SidewaysRight);
        vvLib.moveTillWhiteLineDetect(this, 0.7f);
        Thread.sleep(250);

        vvLib.turnAbsoluteGyroDegrees(this, 90);
        Thread.sleep(250);
        //now we are too far to the side, pull back a bit

        vvLib.moveWheels(this, 2.5f, 0.3f, Backward);

        Thread.sleep(250);
        vvLib.moveWheels(this, 8, 0.3f, SidewaysRight);

        Thread.sleep(250);

        vvLib.moveWheels(this, 50, .3f, SidewaysLeft);
        

        //code for detecting color of the beacon.


        //turn to check color

        /*
        vvLib.turnBeaconArm(this, BeaconServoStateEnum.Look);


        if (vvLib.getBeaconColor(this) == vv_Constants.BeaconColorEnum.RED) {
            //logic for choosing team side red or blue.

            if (TEAM_RED) {
                vvLib.turnBeaconArm(this, BeaconServoStateEnum.Right);
                Thread.sleep(100);
            } else {
                vvLib.turnBeaconArm(this, BeaconServoStateEnum.Left);
                Thread.sleep(100);
            }

        }
        if (vvLib.getBeaconColor(this) == vv_Constants.BeaconColorEnum.BLUE) {
            //logic for choosing team side red or blue.

            if (!TEAM_RED) {
                //we are blue
                vvLib.turnBeaconArm(this, BeaconServoStateEnum.Right);
                Thread.sleep(100);
            } else {
                vvLib.turnBeaconArm(this, BeaconServoStateEnum.Left);
                Thread.sleep(100);
            }

        }
        if (vvLib.getBeaconColor(this) == vv_Constants.BeaconColorEnum.UNKNOWN) {
            vvLib.showBeaconColorValuesOnTelemetry(this, true);
        }


/*
        //now move Sideways left to knock the ball off location.

        vvLib.moveWheels(this, 40, 0.3f, SidewaysLeft);
*/



/*
Touch Sensor is out of plane, so using dead reckoning till fixed.
        //move forward to the beacon panel, until the sensor touches the beacon.
        vvLib.moveTillTouch(this);
        //back off the panel, 1-2 inches to give space to the beacon press.



        vvLib.moveWheels(this, 1.5f, TOUCH_SENSE_POWER, Backward);
        //readjust the orientation again, since the interaction with the beacon face has likely
        //changed the angle of the robot

        vvLib.turnAbsoluteGyroDegrees(this, 90);

        /*

        vvLib.pushAButton(this, Left);
        //wait till the servo reaches the button
        Thread.sleep(1000);
        vvLib.pushAButton(this, Right);
        //wait till the servo reaches the button
        Thread.sleep(1000);
        */


    }


}
