package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.vv_Constants.ARM_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BeaconServoStateEnum.Left;
import static org.firstinspires.ftc.teamcode.vv_Constants.BeaconServoStateEnum.Right;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.Backward;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.Forward;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.INTAKE_MOTOR;
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

        DBG("before try");
        try {
            DBG("Before vvLIb init");
            vvLib = new vv_Lib(this);

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Say", "Hello Driver");    //
            telemetry.update();
            DBG("before waitForStart");
            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            DBG("before Intake");
            vvLib.testMotor(this, INTAKE_MOTOR, 0.3f, 3000);
            DBG("after Intake");
            vvLib.testMotor(this, ARM_MOTOR, 0.3f, 500);
            //DBG("before wormDrive");
            //vvLib.testEncodedMotor(this, wormDriveMotor, 0.3f, 500, 100);

            vvLib.testEncodedMotor(this, FRONT_LEFT_MOTOR, 0.5f, 1000, 1000);
            vvLib.testEncodedMotor(this, FRONT_RIGHT_MOTOR, 0.5f, 1000, 1000);
            vvLib.testEncodedMotor(this, BACK_LEFT_MOTOR, 0.5f, 1000, 1000);
            vvLib.testEncodedMotor(this, BACK_RIGHT_MOTOR, 0.5f, 1000, 1000);


        } catch (vv_Robot.MotorNameNotKnownException MNNKE) {
            telemetryAddData("Motor Not found", "Values:", MNNKE.getMessage());
            telemetryUpdate();
            Thread.sleep(3000);
        } catch (vv_Robot.MotorStalledException MSTE) {
            telemetryAddData("Motor Stalled", "Values:", MSTE.getMessage());
            telemetryUpdate();
            Thread.sleep(3000);
        }


    }

    public void auto1() throws InterruptedException, vv_Robot.MotorNameNotKnownException {

        //Turn the LED on the Color Sensor mounted on the floor of the Robot on
        vvLib.turnFloorColorSensorLedOn(this);

        //test the turn of the robot.

        //clockwise is positive

        //power of 0.5 is recommended to prevent single motor stalls during turns
        //otherwise code has to be written to solve for this (maybe increase margin ?)

        vvLib.moveWheels(this, 12, 0.5f, Forward);
        vvLib.turnAbsoluteGyroDegrees(this, 45);

        //distances are by experimentation.

        vvLib.moveWheels(this, 46, 0.5f, Forward);
        vvLib.turnAbsoluteGyroDegrees(this, 90);

        //move forward to the beacon panel, until the sensor touches the beacon.
        vvLib.moveTillTouch(this);
        //back off the panel, 1-2 inches to give space to the beacon press.

        vvLib.moveWheels(this, 1.5f, TOUCH_SENSE_POWER, Backward);
        //readjust the orientation again, since the interaction with the beacon face has likely
        //changed the angle of the robot

        vvLib.turnAbsoluteGyroDegrees(this, 90);

        vvLib.pushAButton(this, Left);
        //wait till the servo reaches the button
        Thread.sleep(1000);
        vvLib.pushAButton(this, Right);
        //wait till the servo reaches the button
        Thread.sleep(1000);
    }


}