package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_LEFT;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_LOOK_FOR_COLOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_NEUTRAL;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_RIGHT;


@TeleOp(name = "BeaconPositionCalibrationOp", group = "Test")

public class BeaconPositionCalibration extends vv_OpMode {

    vv_Lib vvLib;


    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */


        vvLib = new vv_Lib(this);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Hello Driver", ":I am", ":ready!");    //
        telemetry.addData("Move Robot over line", ":To read", ":values!");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        telemetry.setAutoClear(true);
        for (double i = 0.0f; i < 1.0f; i += 0.1f) {
            vvLib.setBeaconPosition(this, i);
            telemetryAddData("Servo Position", "Value;", "" + vvLib.getBeaconPosition(this));
            telemetryUpdate();
            Thread.sleep(2000);
            idle();
        }

        telemetryAddData("Servo Neutral", "Value;", "" + BEACON_SERVO_NEUTRAL);
        telemetryUpdate();
        vvLib.setBeaconPosition(this, BEACON_SERVO_NEUTRAL);
        Thread.sleep(3000);

        telemetryAddData("Servo Left", "Value;", "" + BEACON_SERVO_LEFT);
        telemetryUpdate();
        vvLib.setBeaconPosition(this, BEACON_SERVO_LEFT);
        Thread.sleep(3000);

        telemetryAddData("Servo Look", "Value;", "" + BEACON_SERVO_LOOK_FOR_COLOR);
        telemetryUpdate();
        vvLib.setBeaconPosition(this, BEACON_SERVO_LOOK_FOR_COLOR);
        Thread.sleep(3000);

        telemetryAddData("Servo Right", "Value;", "" + BEACON_SERVO_RIGHT);
        telemetryUpdate();
        vvLib.setBeaconPosition(this, BEACON_SERVO_RIGHT);
        Thread.sleep(3000);

    }

}


