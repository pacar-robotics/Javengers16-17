package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysRight;


/**
 * Created by Rachael_ on 10/23/2016.
 */


@Autonomous(name = "BasicBlueRightOp", group = "Test")
public class vv_Autonomous extends vv_OpMode
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

        //Turn the LED on the Color Sensor mounted on the floor of the Robot on
        vvLib.turnFloorColorSensorLedOn(this);

        //test the turn of the robot.

        //clockwise is positive

        //power of 0.5 is recommended to prevent single motor stalls during turns
        //otherwise code has to be written to solve for this (maybe increase margin ?)


        vvLib.moveWheels(this, 12, 0.4f, SidewaysRight);
        vvLib.turnAbsoluteGyroDegrees(this, 50);

        //distances are by experimentation.

        vvLib.moveWheels(this, 46, 0.4f, SidewaysRight);
        vvLib.turnAbsoluteGyroDegrees(this, 90);



        /*

        //move forward to the beacon panel, until the sensor touches the beacon.
        vvLib.moveTillTouch(this);
        //back off the panel, 1-2 inches to give space to the beacon press.

        vvLib.moveWheels(this, 1.5f, TOUCH_SENSE_POWER, Backward);
        //readjust the orientation again, since the interaction with the beacon face has likely
        //changed the angle of the robot

        vvLib.turnAbsoluteGyroDegrees(this, 90);

        vvLib.pushAButton(this, Left);
        //wait till the servo reaches the button
        Thread.sleep(1000);
        vvLib.pushAButton(this, Right);
        //wait till the servo reaches the button
        Thread.sleep(1000);

        */
    }


}
