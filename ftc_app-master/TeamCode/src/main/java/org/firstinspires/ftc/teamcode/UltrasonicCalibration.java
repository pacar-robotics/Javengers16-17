package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;


@TeleOp(name = "UltrasonicCalibrationOp", group = "Calibrations")
public class UltrasonicCalibration extends vv_OpMode {

    /* Declare OpMode members. */
    vv_Lib vvLib;
    double readingsArray[];

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryAddData("Initializing", ":Please", "wait..");
        telemetryUpdate();
        DBG("before try");


        readingsArray = new double[9];

        //initialize array
        for (int i = 0; i < 9; i++) {
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


        telemetryAddData("Reading Ultrasonic", ":distance:", ".");
        telemetryUpdate();
        Thread.sleep(2000);

        //looks like the ultrasonic sensor is accurate to about 16cm or 6 inches.
        while (opModeIsActive()) {
            telemetryAddData("Reading Ultrasonic", ":distance:",
                    "" + readUltrasonicDistance() / 2.54); //for inches
            telemetryUpdate();
            idle();

        }





    }

    public double readUltrasonicDistance() throws InterruptedException {

        return filterUltrasonicReadings();

    }

    public double filterUltrasonicReadings() throws InterruptedException {
        //take 9 readings
        for (int i = 0, j = 0; i < 9 && j < 25; i++, j++) {
            readingsArray[i] = vvLib.getFloorUltrasonicReading(this);
            //wait between readings
            Thread.sleep(50);
            if (readingsArray[i] == 0) {
                i--; //bad read, redo. to a maximum of 25 reads
            }
        }
        //Now sort the readings
        Arrays.sort(readingsArray);
        //return the middle element to reduce noise
        return readingsArray[4];
    }

}