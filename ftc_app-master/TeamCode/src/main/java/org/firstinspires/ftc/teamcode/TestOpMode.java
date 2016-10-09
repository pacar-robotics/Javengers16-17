package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp(name="TestOp", group="Test")

public class TestOpMode extends vv_OpMode {

    /* Declare OpMode members. */
    vv_Robot robot           = new vv_Robot();


    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        vv_Lib vvLib = new vv_Lib();

        vvLib.robotInit(this);

        //AGAIN DO NOT CALL THIS METHOD
        vvLib.moveAtAngle(this, 20.0, .5f, 45);
    }

    public void telemetryAddData(String caption, String key, String message){
        telemetry.addLine(caption).addData(key,message);
    }
    public void telemetryUpdate() {
        telemetry.update();
    }

}
