package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_LEFT_PRESSED;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_LEFT_REST;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_RIGHT_PRESSED;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_RIGHT_REST;
import static org.firstinspires.ftc.teamcode.vv_Constants.LEFT_BEACON_BUTTON_SERVO;
import static org.firstinspires.ftc.teamcode.vv_Constants.RIGHT_BEACON_BUTTON_SERVO;


@TeleOp(name = "BeaconPositionCalibrationOp", group = "Calibrations")

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
            vvLib.setBeaconServoPosition(this, LEFT_BEACON_BUTTON_SERVO, i);
            telemetryAddData("Left Beacon Servo Position", "Value;", "" +
                    vvLib.getBeaconServoPosition(this, LEFT_BEACON_BUTTON_SERVO));
            telemetryUpdate();
            Thread.sleep(2000);
            idle();
        }

        telemetry.setAutoClear(true);
        for (double i = 0.0f; i < 1.0f; i += 0.1f) {
            vvLib.setBeaconServoPosition(this, RIGHT_BEACON_BUTTON_SERVO, i);
            telemetryAddData("Right Beacon Servo Position", "Value;", "" +
                    vvLib.getBeaconServoPosition(this, RIGHT_BEACON_BUTTON_SERVO));
            telemetryUpdate();
            Thread.sleep(2000);
            idle();
        }


        telemetryAddData("Left Beacon Servo Rest", "Value;", "" + BEACON_SERVO_LEFT_REST);
        telemetryUpdate();
        vvLib.setBeaconServoPosition(this, LEFT_BEACON_BUTTON_SERVO, BEACON_SERVO_LEFT_REST);
        Thread.sleep(3000);

        telemetryAddData("Left Beacon Servo Pressed", "Value;", "" + BEACON_SERVO_LEFT_PRESSED);
        telemetryUpdate();
        vvLib.pressLeftBeaconButton(this);
        Thread.sleep(3000);

        telemetryAddData("Right Beacon Servo Rest", "Value;", "" + BEACON_SERVO_RIGHT_REST);
        telemetryUpdate();
        vvLib.setBeaconServoPosition(this, RIGHT_BEACON_BUTTON_SERVO, BEACON_SERVO_RIGHT_REST);
        Thread.sleep(3000);

        telemetryAddData("Right Beacon Servo Pressed", "Value;", "" + BEACON_SERVO_RIGHT_PRESSED);
        telemetryUpdate();
        vvLib.pressRightBeaconButton(this);
        Thread.sleep(3000);


    }

}


