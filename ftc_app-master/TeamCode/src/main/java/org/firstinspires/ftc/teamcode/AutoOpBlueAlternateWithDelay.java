package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Kenneth on 2/2/2017.
 */

@Autonomous(name = "BlueCornerAlternate", group = "Autonomous")
public class AutoOpBlueAlternateWithDelay extends vv_OpMode {

    vv_Lib vvLib;

    public void runOpMode() throws InterruptedException {
        telemetryAddData("Initializing, Please wait", "", "");
        telemetryUpdate();

        vvLib = new vv_Lib(this);

        telemetryAddData("<< BLUE ALLIANCE : CORNER TILE>> Ready to go!", "", "");
        telemetryUpdate();

        //Turn the LED on the Color Sensor mounted on the floor of the Robot on
        vvLib.turnFloorColorSensorLedOn(this);

        waitForStart();

        vvLib.blueAlternateAutonomous(this);
    }
}
