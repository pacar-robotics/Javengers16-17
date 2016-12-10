package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.Backward;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.Forward;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.TOUCH_SENSE_POWER;



@TeleOp(name = "ProtoOp", group = "Test")
public class protoOp extends vv_OpMode {

    /* Declare OpMode members. */
    vv_Lib vvLib;

    @Override
    public void runOpMode() throws InterruptedException {


        //Initialize library which in turn initializes the robot plus its hardware map
        //We need to pass the this pointer into vv_Lib in order to call some value added functions
        //in vv_Opmode

        telemetryAddData("Initializing", ":Please", "wait..");
        telemetryUpdate();
        DBG("before try");

            DBG("Before vvLIb init");
            vvLib = new vv_Lib(this);

            // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver", "Im Ready");    //
            telemetry.update();
            DBG("before waitForStart");
            // Wait for the game to start (driver presses PLAY)
            waitForStart();


        vvLib.testMotor(this, FRONT_LEFT_MOTOR, 0.3f, 1000);
        vvLib.testMotor(this, FRONT_RIGHT_MOTOR, 0.3f, 1000);


        vvLib.testMotor(this, BACK_LEFT_MOTOR, 0.3f, 1000);
        vvLib.testMotor(this, BACK_RIGHT_MOTOR, 0.3f, 1000);





    }

    public void auto1() throws InterruptedException, vv_Robot.MotorNameNotKnownException {

        //Turn the LED on the Color Sensor mounted on the floor of the Robot on
        vvLib.turnFloorLightSensorLedOn(this);

        //test the turn of the robot.

        //clockwise is positive

        //power of 0.5 is recommended to prevent single motor stalls during turns
        //otherwise code has to be written to solve for this (maybe increase margin ?)

        vvLib.moveWheels(this, 12, 0.5f, Forward, true);
        vvLib.turnAbsoluteGyroDegrees(this, 45);

        //distances are by experimentation.

        vvLib.moveWheels(this, 46, 0.5f, Forward, true);
        vvLib.turnAbsoluteGyroDegrees(this, 90);

        //back off the panel, 1-2 inches to give space to the beacon press.

        vvLib.moveWheels(this, 1.5f, TOUCH_SENSE_POWER, Backward, true);
        //readjust the orientation again, since the interaction with the beacon face has likely
        //changed the angle of the robot

        vvLib.turnAbsoluteGyroDegrees(this, 90);


        // TO MOVE SIDEWAYS RIGHT:
        /*
        front left - backwards
        back right - backwards
        front right - forwards
        back left - forwards
         */
    }


}