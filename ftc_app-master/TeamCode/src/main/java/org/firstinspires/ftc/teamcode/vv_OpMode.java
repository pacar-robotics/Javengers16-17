package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * Created by thomas on 10/9/2016.
 */

public abstract class vv_OpMode extends LinearOpMode
{
    public void telemetryAddData(String caption, String key, String message){
        telemetry.addLine(caption).addData(key,message);
    }
    public void telemetryUpdate() {
        telemetry.update();
    }

    public void telemetryAddFormattedData(String caption, String key, int number) {
        telemetry.addLine(caption).addData(key, number);
    }

    public void dbg(String key, String message)
    {
        telemetryAddData("Debug", key, message);
        telemetryUpdate();
    }
}
