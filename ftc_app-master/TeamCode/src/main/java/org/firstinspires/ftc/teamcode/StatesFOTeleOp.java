package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;


@TeleOp(name = "FieldOrientedTeleOp", group = "TeleOp")

public class StatesFOTeleOp extends vv_OpMode {

    vv_Lib vvLib;
    vv_TeleLib vvTeleLib;
    CalibFileIO powerFactorFileIO;

    float drivePowerFactor;


    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        telemetry.setAutoClear(true);


        vvLib = new vv_Lib(this);
        vvTeleLib = new vv_TeleLib();
        powerFactorFileIO = new CalibFileIO(this, "PowerFactor");

        try {
            drivePowerFactor = powerFactorFileIO.getCalibrationValue(this);
            telemetryAddData("Power Factor: ", String.valueOf(drivePowerFactor), "");
        } catch (IOException e) {
            telemetryAddData("Problem: ", e.getMessage(), "");
            drivePowerFactor = vv_Constants.STANDARD_DRIVE_POWER_FACTOR;
            telemetryAddData("Power Factor: ", String.valueOf(drivePowerFactor), "");
        } catch (NumberFormatException e) {
            telemetryAddData("Problem: ", e.getMessage(), "");
            drivePowerFactor = vv_Constants.STANDARD_DRIVE_POWER_FACTOR;
            telemetryAddData("Power Factor: ", String.valueOf(drivePowerFactor), "");
        }


        // Send telemetry message to signify robot waiting;
        telemetry.addData("TeleOp Driver", ":I am", ":ready!");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        vvLib.robot.setChooChooToLimit(this);
        while (opModeIsActive()) {
            
            try {

                vvTeleLib.processParticleBallLaunch(this, vvLib);

                vvTeleLib.processFieldOrientedDrive(this, vvLib, drivePowerFactor);

                vvTeleLib.processIntake(this, vvLib);

                vvTeleLib.processLaunchPowerCalibration(this, vvLib);

                vvTeleLib.processBallFlag(this, vvLib);

                vvTeleLib.processCapBallControls(this, vvLib, vvTeleLib );

                vvTeleLib.processFieldOrientedCapBallDrive(this, vvLib, 0.5f);

                vvTeleLib.processYawReset(this, vvLib);

                vvTeleLib.processBeaconOrientationControls(this, vvLib);

                vvTeleLib.processChooChooPosition(this, vvLib);



            } catch (vv_Robot.MotorStalledException MSE) {
                telemetryAddData("Motor Stalled!", "Name", MSE.getMessage());
                telemetryUpdate();
                Thread.sleep(500);
            }

            idle();

        }
    }
}


