package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Rachael_ on 10/23/2016.
 */


@Autonomous(name = "vv_Autonomous", group = "Test")
public class vv_Autonomous extends vv_OpMode {
    vv_Lib vvLib;

    public void runOpMode() throws InterruptedException {
        waitForStart();

        vvLib = new vv_Lib(this);

        vvLib.lineColorSensorOn(this);

        vvLib.moveWheels(this, 60, .3f, vv_Constants.DirectionEnum.Forward);
        vvLib.turnUsingEncoders(this, .3f, 60, vv_Constants.TurnDirectionEnum.Clockwise);
        vvLib.moveUntilLine(this);
        vvLib.moveWheels(this, 60, .3f, vv_Constants.DirectionEnum.Forward);
        vvLib.turnUsingEncoders(this, .3f, 30, vv_Constants.TurnDirectionEnum.Clockwise);
        vvLib.moveTillTouch(this);

        vvLib.displayLineColorSensorLuminosity(this);

    }


}
