package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.Backward;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysRight;


/**
 * Created by Rachael_ on 10/23/2016.
 */


@Autonomous(name = "RedRightNoDelayOp", group = "Horsham")
public class AutoOpRedNoDelay extends vv_OpMode
{
    vv_Lib vvLib;
    vv_Robot vvRobot;
    double readingsArray[];

    public void runOpMode() throws InterruptedException
    {
        telemetryAddData("Initializing, Please wait", "", "");
        telemetryUpdate();
        try {
            vvLib = new vv_Lib(this);

            readingsArray = new double[7];

            //initialize array
            for (int i = 0; i < 7; i++) {
                readingsArray[i] = 0.0f;
            }

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


        //set the ball up and shoot.
        //wind up to autonomous position.
        vvLib.moveWheels(this, 4, .4f, SidewaysRight, true);
        Thread.sleep(250);
        vvLib.turnAbsoluteGyroDegrees(this, -40);
        Thread.sleep(250);


        // Shoot two balls
        vvLib.setupShot(this);
        vvLib.shootBall(this);
        vvLib.setupShot(this);
        //drop ball
        vvLib.dropBall(this);
        vvLib.shootBall(this);
        Thread.sleep(250);

        vvLib.turnAbsoluteGyroDegrees(this, -30);
        Thread.sleep(250);

        vvLib.moveWheels(this, 50, .9f, SidewaysRight, true);
        Thread.sleep(250);

        vvLib.turnAbsoluteGyroDegrees(this, -50);
        Thread.sleep(2000);

        vvLib.turnAbsoluteGyroDegrees(this, -110);
        Thread.sleep(250);

        vvLib.moveWheels(this, 12, 1.0f, Backward, false);
        Thread.sleep(250);
    }


}
