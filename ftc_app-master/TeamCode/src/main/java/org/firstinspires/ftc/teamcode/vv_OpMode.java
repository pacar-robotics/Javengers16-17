package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * Created by thomas on 10/9/2016.
 */

public abstract class vv_OpMode extends LinearOpMode
{

    public static long timer_array[] = new long[5];

    public void DBG(String message) throws InterruptedException {
        if (vv_Constants.DEBUG) {
            telemetry.setAutoClear(vv_Constants.DEBUG_AUTO_CLEAR);
            telemetryAddData("DBG", "Message", ":" + message);
            telemetryUpdate();
            Thread.sleep(vv_Constants.DEBUG_MESSAGE_DISPLAY_TIME);
        }
    }
    public void telemetryAddData(String caption, String key, String message){
        telemetry.addLine(caption).addData(key,message);
    }
    public void telemetryUpdate() {
        telemetry.update();
    }

    public void telemetryAddFormattedData(String caption, String key, int number) {
        telemetry.addLine(caption).addData(key, number);
    }

    public void reset_timer_array(int index) {
        timer_array[index] = System.currentTimeMillis();
    }

    public long time_elapsed_array(int index) {
        //return the time elapsed in milliseconds
        return System.currentTimeMillis() - timer_array[index];
    }

    public interface StopCondition {
        boolean stopCondition(vv_OpMode aOpMode) throws InterruptedException;
    }


}
