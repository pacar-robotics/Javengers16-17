package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;


@TeleOp(name = "UltrasonicPositionTest", group = "Test")
public class UltrasonicPositionTest extends vv_OpMode {

    /* Declare OpMode members. */
    vv_Lib vvLib;
    double readingsArray[];

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryAddData("Initializing", ":Please", "wait..");
        telemetryUpdate();
        DBG("before try");


        readingsArray = new double[3];

        //initialize array
        for (int i = 0; i < 3; i++) {
            readingsArray[i] = 0.0f;
        }

        //Initialize library which in turn initializes the robot plus its hardware map
        //We need to pass the this pointer into vv_Lib in order to call some value added functions
        //in vv_Opmode


        DBG("Before vvLIb init");
        vvLib = new vv_Lib(this);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver", "Im Ready");    //
        telemetry.update();
        DBG("before waitForStart");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        telemetryAddData("Positioning Robot to be 4 inches distance from sensor", ":", ".");
        telemetryUpdate();
        Thread.sleep(2000);

        //looks like the ultrasonic sensor is accurate to about 16cm or 6 inches.
        vvLib.moveSidewaysRight(this, 0.5f);
        while (opModeIsActive() && (readUltrasonicDistance() > 30)) { //in cm
            idle();
        }
        vvLib.stopAllMotors(this);
        Thread.sleep(1000);

        //re-orient

        vvLib.turnAbsoluteGyroDegrees(this, 0);

        while (!vvLib.isBeaconTouchSensorPressed(this)) {
            vvLib.moveSidewaysRight(this, 0.2f);
        }
        vvLib.stopAllMotors(this);
        vvLib.moveWheels(this, 1.0f, 0.4f, vv_Constants.DirectionEnum.SidewaysLeft);
        vvLib.turnAbsoluteGyroDegrees(this, 0);


    }

    public double readUltrasonicDistance() throws InterruptedException {

        return filterUltrasonicReadings();

    }

    public double filterUltrasonicReadings() throws InterruptedException {
        //take 9 readings
        for (int i = 0, j = 0; i < 3 && j < 10; i++, j++) {
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
        return readingsArray[1];
    }

}