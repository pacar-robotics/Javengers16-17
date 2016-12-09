package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


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


        telemetryAddData("Stage 2 Test", "Value;", "Testing color positions");
        telemetryUpdate();
        Thread.sleep(2000);

        telemetryAddData("TEAM BLUE:", "Value;", ":Looking for color");
        telemetryUpdate();
        Thread.sleep(2000);

        if (vvLib.getBeaconColor(this) == vv_Constants.BeaconColorEnum.BLUE) {
            telemetryAddData("TEAM BLUE:", "Value;", ":Found Blue");
            telemetryUpdate();
            Thread.sleep(2000);
            //must press right button
            telemetryAddData("TEAM BLUE:", "Value;", ":Pressing Right Button");
            telemetryUpdate();
            Thread.sleep(2000);
            vvLib.pressRightBeaconButton(this);
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
            vvLib.pressLeftBeaconButton(this);
            Thread.sleep(1000);
        }

        if (vvLib.getBeaconColor(this) == vv_Constants.BeaconColorEnum.UNKNOWN) {
            telemetryAddData("TEAM BLUE:", "Value;", ":Found NO COLOR");
            telemetryUpdate();
            Thread.sleep(2000);
        }


    }

}


