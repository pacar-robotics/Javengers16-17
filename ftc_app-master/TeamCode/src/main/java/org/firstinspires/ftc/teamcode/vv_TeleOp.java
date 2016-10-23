package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * created by Kenneth on 10/1/2016
 */

@TeleOp(name = "TeleOp", group = "TeleOp")

public class vv_TeleOp extends vv_OpMode {

    private float wheelPowerFactor = 1;

    vv_Lib vvLib;

    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        vvLib = new vv_Lib(this);

        while (opModeIsActive()) {

            vvLib.processDriveRobotWithPowerFactor(this, vvLib.powerFactorBasedOnCapBall());

            processBeacon();

            processCapBall();

            processBallCollection(this);

            processShootingAngle();

            processShooting();

            idle();
        }
    }


    private void processBeacon() {
        // Changes Beacon Mechanism to left position in order to score the beacon
        if (gamepad1.dpad_left) {
            vvLib.pushABeaconButton(vv_Constants.BeaconServoStateEnum.Left);
        }
        // Changes Beacon Mechanism to right position in order to score the beacon
        if (gamepad1.dpad_right) {
            vvLib.pushABeaconButton(vv_Constants.BeaconServoStateEnum.Right);
        }
        // Changes Beacon Mechanism to a Neutral Position
        if (gamepad1.dpad_down) {
            vvLib.pushABeaconButton(vv_Constants.BeaconServoStateEnum.Neutral);
        }
    }

    private void processCapBall() throws InterruptedException{
        // Semi-Autonomously Raises the Cap Ball Lift Mechanism to the Scoring Height
        if (gamepad1.y) {
            vvLib.moveCapBallLiftToPosition(this, vv_Constants.CapBallStateEnum.Scoring_Position, 0.5f);
        }
        //Semi-Autonomously Lowers the Cap Ball Lift Mechanism to Resting Position
        if (gamepad1.a) {
            vvLib.moveCapBallLiftToPosition(this, vv_Constants.CapBallStateEnum.Rest, 0.5f);
        }
        //TODO: Change only if we need this
        //Grabs or Releases the Ball [Toggle]
        if (gamepad1.x) {

        }
    }

    private void processBallCollection(vv_OpMode anOp) throws InterruptedException{
        //Changes state of Ball Collection mechanism to Outtake [Toggles On or Off]
        if (gamepad1.right_bumper) {
            vvLib.toggleIntake(anOp);
        }
        //Changes state of Ball Collection mechanism to Outtake [Toggles On or Off]
        if (gamepad1.left_bumper) {
            vvLib.toggleOuttake(anOp);
        }
    }

    private void processShootingAngle() {
        //Changes Angle of the Shooting Mechanism to Position 1
        if (gamepad2.a) {

        }
        //Changes Angle of the Shooting Mechanism to Position 2
        if (gamepad2.x) {

        }
        //Changes Angle of the Shooting Mechanism to Position 3
        if (gamepad2.y) {

        }
        //Changes Angle of the Shooting Mechanism to Position 4
        if (gamepad2.b) {

        }

        //TODO Change the Angle slowly using Joystick Y Values
    }

    private void processShooting() {
        //Loads and Shoots one Ball
        if (gamepad2.left_bumper) {

        }
        //TODO: if we press the button x amount of times, shoot & load the ball x amount of times
    }

    public void telemetryAddData(String caption, String key, String message) {
        telemetry.addLine(caption).addData(key, message);
    }

    public void telemetryUpdate() {
        telemetry.update();
    }

}
