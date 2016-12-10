package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.Forward;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysRight;
import static org.firstinspires.ftc.teamcode.vv_Constants.LAUNCH_POWER_POSITION_REST;


/**
 * Created by Rachael_ on 10/23/2016.
 */


@Autonomous(name = "BlueRightBeaconNoDelayOp", group = "Horsham")
public class AutoOpBlueNoDelayWithBeacon extends vv_OpMode {
    vv_Lib vvLib;
    vv_Robot vvRobot;
    double readingsArray[];

    public void runOpMode() throws InterruptedException {
        telemetryAddData("Initializing, Please wait", "", "");
        telemetryUpdate();
        try {
            vvLib = new vv_Lib(this);

            readingsArray = new double[7];

            //initialize array
            for (int i = 0; i < 7; i++) {
                readingsArray[i] = 0.0f;
            }

            telemetryAddData("Ready to go!", "", "");
            telemetryUpdate();
            //Turn the LED on the Color Sensor mounted on the floor of the Robot on
            vvLib.turnFloorLightSensorLedOn(this);


            waitForStart();


            basic_auto_strategy();


            //  vvLib.moveTillColor(this, vvRobot.getColorSensor(this));
        } catch (vv_Robot.MotorNameNotKnownException MNNKE) {
            telemetryAddData("Motor Not found", "Values:", MNNKE.getMessage());
            telemetryUpdate();
            Thread.sleep(3000);
        }

    }

    public void basic_auto_strategy() throws InterruptedException, vv_Robot.MotorNameNotKnownException {


        //test the turn of the robot.

        //clockwise is positive

        //power of 0.5 is recommended to prevent single motor stalls during turns
        //otherwise code has to be written to solve for this (maybe increase margin ?)


        //set the ball up and shoot.
        //wind up to autonomous position.

        vvLib.setupShot(this);
        vvLib.shootBall(this);
        vvLib.setupShot(this);
        //drop ball
        vvLib.dropBall(this);
        vvLib.shootBall(this);

        try {
            vvLib.setLauncherPowerPosition(this, LAUNCH_POWER_POSITION_REST);
        } catch (vv_Robot.MotorStalledException MSE) {
            telemetryAddData("Motor Stalled!", "Motor Name:", MSE.getMessage());
            telemetryUpdate();
            Thread.sleep(500);
        }
        vvLib.moveWheels(this, 6, 0.2f, SidewaysRight, true); //ramped call

        Thread.sleep(750);
        vvLib.turnAbsoluteGyroDegrees(this, 55);

        Thread.sleep(750);

        //distances are by experimentation.

        vvLib.moveWheels(this, 75, 0.9f, SidewaysRight, true);

        Thread.sleep(1000);
        vvLib.turnAbsoluteGyroDegrees(this, 90);
        Thread.sleep(1000);


        vvLib.moveTillWhiteLineDetect(this, 0.4f, Forward);
        Thread.sleep(500);


        vvLib.turnAbsoluteGyroDegrees(this, 90);
        Thread.sleep(500);

        /*


        vvLib.moveWheels(this, 1.5f, 0.3f, Backward, true);

        Thread.sleep(500);

        //looks like the ultrasonic sensor is accurate to about 16cm or 6 inches.
        vvLib.moveSidewaysRight(this, 0.5f);
        while (opModeIsActive() && (readUltrasonicDistance() > 30)) { //in cm
            idle();
        }
        vvLib.stopAllMotors(this);
        Thread.sleep(1000);

        //re-orient

        vvLib.turnAbsoluteGyroDegrees(this, 90);

        vvLib.stopAllMotors(this);

        vvLib.moveTillWhiteLineDetect(this, 0.3f, Forward);
        vvLib.moveWheels(this, 1.5f, 0.3f, Backward, true);

        vvLib.turnAbsoluteGyroDegrees(this, 90);

        */

        vvLib.moveSidewaysRight(this, 0.20f);
        while (!vvLib.isBeaconTouchSensorPressed(this)) {
            //idle
            idle();
        }
        vvLib.stopAllMotors(this);

        vvLib.turnAbsoluteGyroDegrees(this, 90);


        //vvLib.moveWheels(this, 2.0f, 0.15f, vv_Constants.DirectionEnum.SidewaysRight);


        Thread.sleep(1000);


        //check color

        if (vvLib.getBeaconColor(this) == vv_Constants.BeaconColorEnum.RED) {
            //logic for choosing team side red or blue.

            //we are blue and want to press the red button
            vvLib.pressRightBeaconButton(this);
            Thread.sleep(100);

        } else {
            //we are blue and want to press the blue button
            vvLib.pressLeftBeaconButton(this);
            Thread.sleep(100);
        }

        if (vvLib.getBeaconColor(this) == vv_Constants.BeaconColorEnum.UNKNOWN) {
            vvLib.showBeaconColorValuesOnTelemetry(this, true);
        }


/*
        //now move Sideways left to knock the ball off location.

        vvLib.moveWheels(this, 40, 0.3f, SidewaysLeft);
*/



/*
Touch Sensor is out of plane, so using dead reckoning till fixed.
        //move forward to the beacon panel, until the sensor touches the beacon.
        vvLib.moveTillTouch(this);
        //back off the panel, 1-2 inches to give space to the beacon press.



        vvLib.moveWheels(this, 1.5f, TOUCH_SENSE_POWER, Backward);
        //readjust the orientation again, since the interaction with the beacon face has likely
        //changed the angle of the robot

        vvLib.turnAbsoluteGyroDegrees(this, 90);

        /*

        vvLib.pushAButton(this, Left);
        //wait till the servo reaches the button
        Thread.sleep(1000);
        vvLib.pushAButton(this, Right);
        //wait till the servo reaches the button
        Thread.sleep(1000);
        */


    }

    public double readUltrasonicDistance() throws InterruptedException {

        return filterUltrasonicReadings();

    }

    public double filterUltrasonicReadings() throws InterruptedException {
        //take 9 readings
        for (int i = 0, j = 0; i < 7 && j < 10; i++, j++) {
            readingsArray[i] = vvLib.getFloorUltrasonicReading(this);
            //wait between readings
            Thread.sleep(20);
            if (readingsArray[i] == 0) {
                i--; //bad read, redo. to a maximum of 10 reads
            }
        }
        //Now sort the readings
        Arrays.sort(readingsArray);
        //return the middle element to reduce noise
        return readingsArray[3];
    }


}
