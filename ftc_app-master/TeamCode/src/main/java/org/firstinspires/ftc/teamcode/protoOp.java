package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Proto", group="Test")


public class protoOp extends vv_OpMode {

    /* Declare OpMode members. */


    @Override
    public void runOpMode() throws InterruptedException {


        //Initialize library which in turn initializes the robot plus its hardware map
        //We need to pass the this pointer into vv_Lib in order to call some value added functions
        //in vv_Opmode


        vv_Lib vvLib = new vv_Lib(this);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Turn the LED on the Color Sensor mounted on the floor of the Robot on
        vvLib.turnFloorColorSensorLedOn(this);

        //test the turn of the robot.

        //clockwise is positive

        //power of 0.5 is recommended to prevent single motor stalls during turns
        //otherwise code has to be written to solve for this (maybe increase margin ?)

        vvLib.moveWheels(this, 12, 0.5f, vv_Constants.DirectionEnum.Forward);
        vvLib.turnAbsoluteGyroDegrees(this, 45);

        //distances are by experimentation.

        vvLib.moveWheels(this, 46, 0.5f, vv_Constants.DirectionEnum.Forward);
        vvLib.turnAbsoluteGyroDegrees(this, 90);

        //move forward to the beacon panel, until the sensor touches the beacon.
        vvLib.moveTillTouch(this);
        //back off the panel, 1-2 inches to give space to the beacon press.

        vvLib.moveWheels(this, 1.5f, vv_Constants.TOUCH_SENSE_POWER, vv_Constants.DirectionEnum.Backward);
        //readjust the orientation again, since the interaction with the beacon face has likely
        //changed the angle of the robot

        vvLib.turnAbsoluteGyroDegrees(this, 90);

        vvLib.pushAButton(this, vv_Constants.ButtonEnum.Left);
        //wait till the servo reaches the button
        Thread.sleep(1000);
        vvLib.pushAButton(this, vv_Constants.ButtonEnum.Right);
        //wait till the servo reaches the button
        Thread.sleep(1000);
    }


}
