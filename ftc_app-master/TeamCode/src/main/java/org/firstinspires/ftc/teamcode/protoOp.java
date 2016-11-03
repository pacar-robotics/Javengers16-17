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

        //test the turn of the robot.

        //clockwise is positive
        
        vvLib.turnGyroDegrees(this, 90);
        Thread.sleep(2000);
        vvLib.turnGyroDegrees(this, -90);
        Thread.sleep(2000);
        vvLib.turnGyroDegrees(this, 45);
        Thread.sleep(2000);
        vvLib.turnGyroDegrees(this, -180);
        Thread.sleep(2000);


        while (opModeIsActive()){
            //Send Luminosity data to Telemetry
            vvLib.showFloorColorSensorLumnosityOnTelemetry(this, false);
            //Send Gyro Heading data to Telemetry
            vvLib.showBaseGyroSensorHeadingOnTelemetry(this, false);
            //Send Gyro Integrated Z value to Telemetry
            //this time flush the display and show all values.
            vvLib.showBaseGyroSensorIntegratedZValueOnTelemetry(this, true);
            idle();

        }
        //Initialize library which in turn initializes the robot plus its hardware map
        //We need to pass the this pointer into vv_Lib in order to call some value added functions
        //in vv_Opmode

    }
}
