package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Rachael_ on 10/23/2016.
 */


@Autonomous(name="vv_Autonomous", group="Test")
public class vv_Autonomous extends vv_OpMode
{
    vv_Lib vvLib;
    vv_Robot vvRobot;

    public void runOpMode() throws InterruptedException
    {
        try {
            waitForStart();

            vvLib = new vv_Lib(this);
            vvRobot = new vv_Robot();

            //  vvLib.moveTillColor(this, vvRobot.getColorSensor(this));
        } catch (vv_Robot.MotorNameNotKnownException MNNKE) {
            telemetryAddData("Motor Not found", "Values:", MNNKE.getMessage());
            telemetryUpdate();
            Thread.sleep(3000);
        }

    }


}
