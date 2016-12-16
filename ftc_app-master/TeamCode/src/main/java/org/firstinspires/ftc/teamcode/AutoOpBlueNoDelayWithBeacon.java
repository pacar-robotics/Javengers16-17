package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;

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
        lineDetectCondition condition = new lineDetectCondition();
        vvLib.universalMoveRobotByAxisVelocity(this, 0.3, -0.4, 0.0, 7000, condition);
        //rotate to face beacon press mode
        vvLib.turnAbsoluteGyroDegrees(this, 92); //with trim
        vvLib.moveWheels(this, 5, 0.2f, Backward, true); // adjust face position to match beacons
        double distanceToBeaconWall = vvLib.getFloorUltrasonicReading(this) / 2.54; //in inches
        vvLib.turnAbsoluteGyroDegrees(this, 92); //with trim
        //now try moving that distance, adjusting for inset of ultrasonic sensor
        vvLib.moveWheels(this, (float) (distanceToBeaconWall - 4.5), 0.2f, SidewaysRight, true);



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

//conditions that can stop the robot.


    public class lineDetectCondition implements vv_OpMode.StopCondition {
        public boolean StopCondition(vv_OpMode aOpMode) throws InterruptedException {
            return ((vvLib.getFloorLightIntensity(aOpMode) >= FLOOR_WHITE_THRESHOLD));
        }
    }


}
