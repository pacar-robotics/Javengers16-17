package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.vv_Constants.TRIGGER_THRESHOLD;


@TeleOp(name = "HorshamTeleOp", group = "Horsham")

public class HorshamTeleOp extends vv_OpMode {

    vv_Lib vvLib;


    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        telemetry.setAutoClear(true);


        vvLib = new vv_Lib(this);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Hello Driver", ":I am", ":ready!");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        waitForStart();


        while (opModeIsActive()) {


            processLaunch();

            vvLib.driveRobotWithPowerFactor(this, 0.6f);

            processIntake();

            processLaunchPowerCalibration();

            idle();

        }
    }


    private void processIntake()
            throws InterruptedException {
        //Changes state of Ball Collection mechanism to Outtake [Toggles On or Off]
        if (gamepad1.right_bumper) {
            vvLib.toggleIntake(this);
            Thread.sleep(250);
        }
        //Changes state of Ball Collection mechanism to Outtake [Toggles On or Off]
        if (gamepad1.left_bumper) {
            vvLib.toggleOuttake(this);
            Thread.sleep(250);
        }
    }

    private void processLaunch() throws InterruptedException {
        if (gamepad1.a) {
            //shoot a ball
            //then setup for next shot.
            //launch where we are.
            //used to calibrate the location.

            vvLib.dropBall(this);
            vvLib.shootBall(this);
            vvLib.setupShot(this);
        }
    }

    private void processBeacon() throws InterruptedException {
        // Changes Beacon Mechanism to left position in order to score the beacon
        if (gamepad2.left_bumper) {
            vvLib.pressLeftBeaconButton(this);
        }
        // Changes Beacon Mechanism to right position in order to score the beacon
        if (gamepad2.right_bumper) {
            vvLib.pressRightBeaconButton(this);
        }

    }


    /**
     * Changes the power of the Shooting Mechanism
     * <p>
     * gamepad2.a = changes the mechanism to position 1 (___ Tiles away) gamepad2.x = changes the
     * mechanism to position 2 (___ Tiles away) gamepad2.y = changes the mechanism to position 3
     * (___ Tiles away) gamepad2.b = changes the mechanism to position 4 (___ Tiles away)
     */


    private void processLaunchPowerCalibration() throws InterruptedException {

        try {
            telemetry.setAutoClear(true);
            telemetryAddData("Power Position Before:", "Value", ":" +
                    vvLib.getLauncherPowerPosition(this));
            telemetryUpdate();

            //read the triggers from game pad.
            if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
                vvLib.decreaseLauncherPower(this);
            }
            if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
                vvLib.increaseLauncherPower(this);
            }
            telemetryAddData("Power Position After:", "Value", ":" +
                    vvLib.getLauncherPowerPosition(this));
            telemetryUpdate();
        } catch (vv_Robot.MotorStalledException MSE) {
            telemetryAddData("Motor Stalled!", "Name", MSE.getMessage());
            telemetryUpdate();
            Thread.sleep(500);
        }
    }
}