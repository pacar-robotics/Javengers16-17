package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Class vv_TeleOp is the class that contains the runOpMode () which runs during the Tele Op session
 * in the 2016-2017 FTC Season. It runs a series of processes which controls certain mechanisms in
 * the robot when a certain button is pressed.
 *
 * @author Kenneth Kannampully
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

            processBeacon(this);

            processCapBall(this);

            processBallCollection(this);

            processShootingAngle(this);

            processShooting(this);

            idle();
        }
    }


    /**
     * Changes the state of the Beacon Button Mechanism depending on the D-PAD Button Pressed
     *
     * gamepad1.dpad_left = changes mechanism to left position gamepad1.dpad_right = changes
     * mechanism to right position gamepad1.dpad_down = changes mechanism to neutral position
     */
    private void processBeacon(vv_OpMode anOpMode) {
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

    /**
     * Controls the Cap Ball Mechanism by either raising the lift, lowering the lift, or grabbing
     * the ball
     *
     * gamepad1.y = raises the lift to scoring position gamepad1.a = lowers the lift to the resting
     * position gamepad.x = graps/releases the cap ball
     */
    private void processCapBall(vv_OpMode anOpMode) throws InterruptedException {
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

    /**
     * Controls the Ball Collector Motor
     *
     * gamepad1.right_bumper = turns the motor either to off or to the intake power
     * gamepad1.left_bumper = turns the motor either to off or to the outtake power
     *
     * @param anOp an object of vv_OpMode
     */
    private void processBallCollection(vv_OpMode anOp) throws InterruptedException {
        //Changes state of Ball Collection mechanism to Outtake [Toggles On or Off]
        if (gamepad1.right_bumper) {
            vvLib.toggleIntake(anOp);
        }
        //Changes state of Ball Collection mechanism to Outtake [Toggles On or Off]
        if (gamepad1.left_bumper) {
            vvLib.toggleOuttake(anOp);
        }
    }

    /**
     * Changes the angle of the Shooting Mechanism
     *
     * gamepad2.a = changes the mechanism to position 1 (___ Tiles away) gamepad2.x = changes the
     * mechanism to position 2 (___ Tiles away) gamepad2.y = changes the mechanism to position 3
     * (___ Tiles away) gamepad2.b = changes the mechanism to position 4 (___ Tiles away)
     *
     * @param anOpMode an object of vv_OpMode
     */
    private void processShootingAngle(vv_OpMode anOpMode) {
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

    /**
     * If the gamepad2.left_bumper is pressed x amount of times, the mechanism shoots the ball x
     * amount of times
     *
     * @param anOpMode object of vv_OpMode
     */
    private void processShooting(vv_OpMode anOpMode) {
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
