package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_LEFT;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_LOOK_FOR_COLOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_NEUTRAL;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_RIGHT;


@TeleOp(name = "BeaconTestOp", group = "Test")

public class BeaconTest extends vv_OpMode {

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

        telemetryAddData("Stage 1 Test", "Value;", "Testing static position");
        telemetryUpdate();

        telemetryAddData("Servo Neutral", "Value;", "" + BEACON_SERVO_NEUTRAL);
        telemetryUpdate();
        vvLib.setBeaconPosition(this, BEACON_SERVO_NEUTRAL);
        Thread.sleep(3000);

        telemetryAddData("Servo Left", "Value;", "" + BEACON_SERVO_LEFT);
        telemetryUpdate();
        vvLib.setBeaconPosition(this, BEACON_SERVO_LEFT);
        Thread.sleep(1000);

        telemetryAddData("Servo Look", "Value;", "" + BEACON_SERVO_LOOK_FOR_COLOR);
        telemetryUpdate();
        vvLib.setBeaconPosition(this, BEACON_SERVO_LOOK_FOR_COLOR);
        Thread.sleep(1000);

        telemetryAddData("Servo Right", "Value;", "" + BEACON_SERVO_RIGHT);
        telemetryUpdate();
        vvLib.setBeaconPosition(this, BEACON_SERVO_RIGHT);
        Thread.sleep(1000);

        telemetryAddData("Stage 2 Test", "Value;", "Testing color positions");
        telemetryUpdate();
        Thread.sleep(2000);

        telemetryAddData("TEAM BLUE:", "Value;", ":Looking for color");
        telemetryUpdate();
        vvLib.setBeaconPosition(this, BEACON_SERVO_LOOK_FOR_COLOR);
        Thread.sleep(2000);

        if (vvLib.getBeaconColor(this) == vv_Constants.BeaconColorEnum.BLUE) {
            telemetryAddData("TEAM BLUE:", "Value;", ":Found Blue");
            telemetryUpdate();
            Thread.sleep(2000);
            //must press right button
            telemetryAddData("TEAM BLUE:", "Value;", ":Pressing Right Button");
            telemetryUpdate();
            Thread.sleep(2000);
            vvLib.setBeaconPosition(this, BEACON_SERVO_RIGHT);
            Thread.sleep(1000);
        }

        if (vvLib.getBeaconColor(this) == vv_Constants.BeaconColorEnum.RED) {
            telemetryAddData("TEAM BLUE:", "Value;", ":Found RED");
            telemetryUpdate();
            Thread.sleep(2000);
            //must press right button
            telemetryAddData("TEAM BLUE:", "Value;", ":Pressing LEFT Button");
            telemetryUpdate();
            Thread.sleep(2000);
            vvLib.setBeaconPosition(this, BEACON_SERVO_LEFT);
            Thread.sleep(1000);
        }

        if (vvLib.getBeaconColor(this) == vv_Constants.BeaconColorEnum.UNKNOWN) {
            telemetryAddData("TEAM BLUE:", "Value;", ":Found NO COLOR");
            telemetryUpdate();
            Thread.sleep(2000);
        }


    }

}


