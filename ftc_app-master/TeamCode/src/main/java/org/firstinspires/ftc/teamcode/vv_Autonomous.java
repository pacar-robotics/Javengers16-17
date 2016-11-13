package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Rachael_ on 10/23/2016.
 */


@Autonomous(name = "vv_Autonomous", group = "Test")
public class vv_Autonomous extends vv_OpMode {
    vv_Lib vvLib;

    public void runOpMode() throws InterruptedException {

        vvLib = new vv_Lib(this);

        // Distance between lines = 114 cm

        //vvLib.turnUsingGyro(this, 90, vv_Constants.TurnDirectionEnum.Clockwise);

        //Thread.sleep(2000);

        waitForStart();


        vvLib.lineColorSensorOn(this);


        /*
        vvLib.moveWheels(this, 20.32f, .5f, vv_Constants.DirectionEnum.Forward);
        vvLib.turnUsingGyro(this, 45, vv_Constants.TurnDirectionEnum.Clockwise);
        vvLib.moveWheels(this, 100, .4f, vv_Constants.DirectionEnum.Forward);
        vvLib.turnUsingGyro(this, 44, vv_Constants.TurnDirectionEnum.Clockwise);
        vvLib.moveUntilLine(this);
       // vvLib.turnUsingEncoders(this, .4f, 5, vv_Constants.TurnDirectionEnum.Counterclockwise);
        vvLib.moveTillTouch(this);
        vvLib.moveWheels(this, 4, .3f, vv_Constants.DirectionEnum.Backward);
        vvLib.pushAButton(this, vv_Constants.ButtonEnum.Left);
        vvLib.pushAButton(this, vv_Constants.ButtonEnum.Right);

        vvLib.displayLineColorSensorLuminosity(this);

        */


//        vvLib.turnAbsoluteUsingGyro(this, 90, vv_Constants.TurnDirectionEnum.Clockwise);
//        Thread.sleep(500);
        vvLib.turnAbsoluteUsingGyro(this, 360);
        vvLib.turnAbsoluteUsingGyro(this, 90);
        vvLib.turnAbsoluteUsingGyro(this, 45);



    }


}
