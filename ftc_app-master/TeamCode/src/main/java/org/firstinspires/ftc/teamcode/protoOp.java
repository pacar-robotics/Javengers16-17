package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Proto", group="Test")


public class protoOp extends vv_OpMode {

    /* Declare OpMode members. */


    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        vv_Lib vvLib = new vv_Lib(this);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Turn the LED on the Color Sensor mounted on the floor of the Robot on
        vvLib.turnFloorColorSensorLedOn(this);

        while (opModeIsActive()){
            //Send Luminosity data to Telemetry
            vvLib.showFloorColorSensorLumnosityOnTelemetry(this);
            idle();
        }
        //Initialize library which in turn initializes the robot plus its hardware map
        //We need to pass the this pointer into vv_Lib in order to call some value added functions
        //in vv_Opmode

    }
}
