package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Kenneth on 2/2/2017.
 */

@Autonomous(name = "RedCornerAlternate", group = "Autonomous")
public class AutoOpRedAlternateWithDelay extends vv_OpMode {

    vv_Lib vvLib;

    public void runOpMode() throws InterruptedException {
        telemetryAddData("Initializing, Please wait", "", "");
        telemetryUpdate();

        vvLib = new vv_Lib(this);

        telemetryAddData("<< RED ALLIANCE : CORNER TILE>> Ready to go!", "", "");
        telemetryUpdate();

        //Turn the LED on the Color Sensor mounted on the floor of the Robot on
        vvLib.turnFloorLightSensorLedOn(this);

        waitForStart();

        vvLib.redAlternateAutonomous(this);
    }
}
