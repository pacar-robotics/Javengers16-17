package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.Backward;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysRight;
import static org.firstinspires.ftc.teamcode.vv_Constants.FLOOR_WHITE_THRESHOLD;


/**
 * Created by Rachael_ on 10/23/2016.
 */


@Autonomous(name = "BlueRightBeaconNoDelayOp", group = "HorshamTest")
public class AutoOpBlueNoDelayWithBeacon extends vv_OpMode {
    vv_Lib vvLib;
    vv_Robot vvRobot;
    double readingsArray[];

    public void runOpMode() throws InterruptedException {
        telemetryAddData("Initializing, Please wait", "", "");
        telemetryUpdate();

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



    }

    public void basic_auto_strategy() throws InterruptedException {


        //lets move the robot at 45 degrees till we detect the line.
        lineDetectCondition ldCondition = new lineDetectCondition();
        //move the robot with no pulsed movement
        vvLib.universalMoveRobotByAxisVelocity(this, 0.3, -0.4, 0.0, 7000, ldCondition, false, 0, 0);
        //rotate to face beacon press mode
        vvLib.turnAbsoluteGyroDegrees(this, 92); //with trim
        vvLib.moveWheels(this, 5, 0.2f, Backward, true); // adjust face position to match beacons
        //read distance from ultrasonic sensor, noise filtered, with 7 readings in a set.

        double distanceToBeaconWall = vvLib.getFloorUltrasonicReading(this, 7) / 2.54; //in inches

        vvLib.turnAbsoluteGyroDegrees(this, 92); //with trim
        //now try moving that distance, adjusting for inset of ultrasonic sensor
        //move toward the beacons but stop short (approx 1.5 inches short).
        vvLib.moveWheels(this, (float) (distanceToBeaconWall - 6), 0.2f, SidewaysRight, true);

        //lets do a pulse move until the beacon touch sensor is pressed
        beaconTouchSensorPressedCondition btspCondition = new beaconTouchSensorPressedCondition();
        //run for 200 ms, rest for 100, max of 7000 ms, until the beaconTouchSensor is pressed
        vvLib.universalMoveRobotByAxisVelocity(this, 0.2, 0, 0.0, 7000, btspCondition, true, 100, 100);


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


}
