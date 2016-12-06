package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="AutoTest", group="Test")


public class TestOpMode extends vv_OpMode {

    /* Declare OpMode members. */


    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        vv_Lib vvLib = new vv_Lib(this);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        vvLib.test_Motors(this);



        while (opModeIsActive()){
            //vvLib.setupShot(this);

            if(gamepad1.x){

                vvLib.moveWormDriveMotorToPosition(this, vv_Constants.SpringPositionsEnum.Position1);
            }
            else if(gamepad1.y){

                vvLib.moveWormDriveMotorToPosition(this, vv_Constants.SpringPositionsEnum.Position2);
            }
            else if(gamepad1.b){

                vvLib.moveWormDriveMotorToPosition(this, vv_Constants.SpringPositionsEnum.Position3);
            }

            idle();
        }
        //Initialize library which in turn initializes the robot plus its hardware map
        //We need to pass the this pointer into vv_Lib in order to call some value added functions
        //in vv_Opmode

    }
}
