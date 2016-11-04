package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Rachael_ on 10/23/2016.
 */


@Autonomous(name="vv_Autonomous", group="Test")
public class vv_Autonomous extends vv_OpMode
{
    vv_Lib vvLib;

    public void runOpMode() throws InterruptedException
    {

        vvLib.moveTillColor(this, vvLib.robot.getColorSensor(this));

        waitForStart();

        vvLib = new vv_Lib(this);
        //vvLib.robot.getColorSensor(this).enableLed(true);
        //Thread.sleep(15000);
    }


}
