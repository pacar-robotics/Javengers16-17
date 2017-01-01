package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.Backward;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysLeft;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysRight;
import static org.firstinspires.ftc.teamcode.vv_Constants.EOPD_PROXIMITY_THRESHOLD;
import static org.firstinspires.ftc.teamcode.vv_Constants.FLOOR_WHITE_THRESHOLD;


/**
 * Created by Rachael_ on 10/23/2016.
 */


@Autonomous(name = "BlueLeftBeaconNoDelayOp", group = "HorshamTest")
public class AutoOpBlueLeftNoDelayWithBeacon extends vv_OpMode {
    vv_Lib vvLib;
    vv_Robot vvRobot;
    double readingsArray[];

    public void runOpMode() throws InterruptedException {
        telemetryAddData("Initializing, Please wait", "", "");
        telemetryUpdate();

        vvLib = new vv_Lib(this);


        telemetryAddData("Ready to go!", "", "");
        telemetryUpdate();
        //Turn the LED on the Color Sensor mounted on the floor of the Robot on
        vvLib.turnFloorLightSensorLedOn(this);


        waitForStart();


        basic_auto_strategy();


    }

    public void basic_auto_strategy() throws InterruptedException {


        //lets move the robot at approx 60 degrees till we detect the line.


        //move the robot with no pulsed movement for a max duration of 3.5 seconds.
        //we want to detect the line orthogonally not at a smaller variable angle for
        //consistency in positioning.
        //duration is found by estimation and test.


        //first shoot two balls.

        //** disabled to allow for mechanical fixes to hold the choo - choo to frame
        //12/31/2016
        /*

        // Shoot the first ball
        vvLib.shootBall(this);
        vvLib.setupShot(this);

        //drop ball
        vvLib.dropBall(this);

        //Shoot the second ball.
        vvLib.shootBall(this);
        Thread.sleep(50);
        */



        falseCondition falseCondition = new falseCondition();

        vvLib.universalMoveRobotByAxisVelocity(this, 0.35, -0.55, 0.0, 3500, falseCondition, false, 0, 0);

        vvLib.turnAbsoluteMxpGyroDegrees(this, 90); //with trim

        //now detect the line but at right angles
        //for first beacon

        lineDetectCondition ldCondition = new lineDetectCondition();

        vvLib.universalMoveRobotByAxisVelocity(this, 0.0, 0.4, 0.0, 3000, ldCondition, false, 0, 0);
        //now detect the line but at right angles

        Thread.sleep(50);

        vvLib.moveWheels(this, 3.25f, 0.8f, Backward, true); // adjust face position to match beacons

        Thread.sleep(50);

        vvLib.turnAbsoluteMxpGyroDegrees(this, 90); //with trim, readjust to prep for ultrasomic read

        //read distance from ultrasonic sensor, noise filtered, with 7 readings in a set.
        double distanceToBeaconWall = vvLib.getFloorUltrasonicReading(this, 7) / 2.54; //in inches


        //now try moving that distance, adjusting for inset of ultrasonic sensor
        //move toward the beacons but stop short (approx 1.5 inches short).
        vvLib.moveWheels(this, (float) (distanceToBeaconWall - 6), 0.8f, SidewaysRight, true);

        //lets do a pulse move until the beacon touch sensor is pressed
        eopdProximityOrDistanceClosedCondition epdcCondition = new eopdProximityOrDistanceClosedCondition();
        //run for 200 ms, rest for 100, max of 7000 ms, until the beaconTouchSensor is pressed

        vvLib.universalMoveRobotByAxisVelocity(this, 0.2, 0, 0.0, 1500, epdcCondition, true, 50, 100);

        //now sense beacon color and press beacon

        vvLib.detectColorAndPressBeacon(this, vv_Constants.BeaconColorEnum.BLUE);

        //now to work on second beacon.

        Thread.sleep(50);

        //now move to second beacon

        //pull back 10 inches to create clearance
        vvLib.moveWheels(this, 10, 0.8f, SidewaysLeft, true);

        //orient to 90 degrees to field

        Thread.sleep(50);

        vvLib.turnAbsoluteMxpGyroDegrees(this, 87); //with trim


        //lets move over the first beacon line, to prevent stopping at wrong line.

        //vvLib.moveWheels(this, 40.0f, 0.9f, Forward, true);

        vvLib.universalMoveRobotByAxisVelocity(this, 0.0, 0.8, 0.0, 1200, falseCondition, false, 0, 0);



        //move till detect second beacon
        vvLib.universalMoveRobotByAxisVelocity(this, 0.0, 0.3, 0.0, 3000, ldCondition, false, 0, 0);
        //now detect the line but at right angles

        Thread.sleep(50);

        vvLib.moveWheels(this, 3.25f, 0.8f, Backward, true); // adjust face position to match beacons

        Thread.sleep(50);

        vvLib.turnAbsoluteMxpGyroDegrees(this, 90); //with trim, readjust to prep for ultrasomic read

        //read distance from ultrasonic sensor, noise filtered, with 7 readings in a set.
        distanceToBeaconWall = vvLib.getFloorUltrasonicReading(this, 7) / 2.54; //in inches


        //now try moving that distance, adjusting for inset of ultrasonic sensor
        //move toward the beacons but stop short (approx 1.5 inches short).
        vvLib.moveWheels(this, (float) (distanceToBeaconWall - 6), 0.8f, SidewaysRight, true);

        //lets do a pulse move until the eopd proximity is triggered

        //run for 200 ms, rest for 100, max of 7000 ms, until the beaconTouchSensor is pressed
        vvLib.turnAbsoluteMxpGyroDegrees(this, 90); //

        vvLib.universalMoveRobotByAxisVelocity(this, 0.2, 0, 0.0, 1500, epdcCondition, true, 50, 100);

        //now sense beacon color and press beacon

        vvLib.detectColorAndPressBeacon(this, vv_Constants.BeaconColorEnum.BLUE);


        Thread.sleep(50);


    }


//conditions that can stop the robot.


    public class lineDetectCondition implements vv_OpMode.StopCondition {
        public boolean StopCondition(vv_OpMode aOpMode) throws InterruptedException {
            return ((vvLib.getFloorLightIntensity(aOpMode) >= FLOOR_WHITE_THRESHOLD));
        }
    }

    public class beaconTouchSensorPressedCondition implements vv_OpMode.StopCondition {
        public boolean StopCondition(vv_OpMode aOpMode) throws InterruptedException {
            return (vvLib.isBeaconTouchSensorPressed(aOpMode));
        }
    }

    public class falseCondition implements vv_OpMode.StopCondition {
        //can be used as an empty condition, so the robot keeps running in universal movement
        public boolean StopCondition(vv_OpMode aOpMode) throws InterruptedException {
            return (false);
        }
    }

    public class beaconTouchOrDistanceClosedCondition implements vv_OpMode.StopCondition {
        public boolean StopCondition(vv_OpMode aOpMode) throws InterruptedException {
            return (vvLib.isBeaconTouchSensorPressed(aOpMode) ||
                    ((vvLib.getFloorUltrasonicReading(aOpMode, 7) / 2.54) < 5));
        }
    }

    public class eopdProximityOrDistanceClosedCondition implements vv_OpMode.StopCondition {
        public boolean StopCondition(vv_OpMode aOpMode) throws InterruptedException {
            return ((vvLib.getEopdRawValue(aOpMode) > EOPD_PROXIMITY_THRESHOLD) ||
                    ((vvLib.getFloorUltrasonicReading(aOpMode, 7) / 2.54) < 5));
        }
    }

}
