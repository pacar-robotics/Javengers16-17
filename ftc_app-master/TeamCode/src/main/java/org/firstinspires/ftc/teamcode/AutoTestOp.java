package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Rachael_ on 10/23/2016.
 */


@Autonomous(name = "AutoTest2", group = "Test")
public class AutoTestOp extends vv_OpMode {
    vv_Lib vvLib;

    public void runOpMode() throws InterruptedException {
        waitForStart();

        vvLib = new vv_Lib(this);

        vvLib.lineColorSensorOn(this);


        vvLib.turnUsingEncoders(this, .5f, 45, vv_Constants.TurnDirectionEnum.Clockwise);
        vvLib.moveWheels(this, 100, .4f, vv_Constants.DirectionEnum.Forward);
        vvLib.turnUsingEncoders(this, .5f, 44, vv_Constants.TurnDirectionEnum.Clockwise);
        vvLib.moveUntilLine(this);
        vvLib.turnUsingEncoders(this, .4f, 5, vv_Constants.TurnDirectionEnum.Counterclockwise);
        vvLib.moveTillTouch(this);
        vvLib.moveWheels(this, 4, .4f, vv_Constants.DirectionEnum.Backward);
        vvLib.pushAButton(this, vv_Constants.ButtonEnum.Left);
        vvLib.pushAButton(this, vv_Constants.ButtonEnum.Right);

        vvLib.displayLineColorSensorLuminosity(this);

    }


}
