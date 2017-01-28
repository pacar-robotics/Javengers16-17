package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "YorkFieldOrientedTeleOp", group = "TeleOp")

public class YorkFOTeleOp extends vv_OpMode {

    vv_Lib vvLib;
    vv_TeleLib vvTeleLib;


    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        telemetry.setAutoClear(true);


        vvLib = new vv_Lib(this);
        vvTeleLib = new vv_TeleLib();


        // Send telemetry message to signify robot waiting;
        telemetry.addData("TeleOp Driver", ":I am", ":ready!");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        waitForStart();


        while (opModeIsActive()) {


            try {
                vvTeleLib.processParticleBallLaunch(this, vvLib);

                vvTeleLib.processFieldOrientedDrive(this, vvLib, 0.6f);

                vvTeleLib.processIntake(this, vvLib);

                vvTeleLib.processLaunchPowerCalibration(this, vvLib);

                vvTeleLib.processBallFlag(this, vvLib);

                vvTeleLib.processCapBallControls(this, vvLib);

                vvTeleLib.processFieldOrientedCapBallDrive(this, vvLib, 0.2f);

                vvTeleLib.processYawReset(this, vvLib);

                vvTeleLib.processBeaconOrientationControls(this, vvLib);

            } catch (vv_Robot.MotorStalledException MSE) {
                telemetryAddData("Motor Stalled!", "Name", MSE.getMessage());
                telemetryUpdate();
                Thread.sleep(500);
            }

            idle();

        }
    }
}


