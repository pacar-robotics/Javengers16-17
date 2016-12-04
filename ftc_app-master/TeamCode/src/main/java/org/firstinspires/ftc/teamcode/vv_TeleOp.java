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

            vvLib.processDriveRobotWithPowerFactor(this, 0.9f);

//            vvLib.powerFactorBasedOnCapBall() //TODO: ADD THIS

            processBeacon(this);

//            processCapBall(this);
//
            processBallCollection(this);
//
//            processShootingAngle(this);
//
//            processShooting(this);

            idle();
        }
    }


    /**
     * Changes the state of the Beacon Button Mechanism depending on the D-PAD Button Pressed
     *
     * gamepad2.dpad_left = changes mechanism to left position gamepad2.dpad_right = changes
     * mechanism to right position gamepad2.dpad_down = changes mechanism to neutral position
     */
    private void processBeacon(vv_OpMode anOpMode) {
        // Changes Beacon Mechanism to left position in order to score the beacon
        if (gamepad2.dpad_left) {
            vvLib.pushABeaconButton(anOpMode, vv_Constants.BeaconServoStateEnum.Left);
        }
        // Changes Beacon Mechanism to right position in order to score the beacon
        if (gamepad2.dpad_right) {
            vvLib.pushABeaconButton(anOpMode, vv_Constants.BeaconServoStateEnum.Right);
        }
        // Changes Beacon Mechanism to a Neutral Position
        if (gamepad2.dpad_down) {
            vvLib.pushABeaconButton(anOpMode, vv_Constants.BeaconServoStateEnum.Neutral);
        }
    }

    /**
     * Sets power of CapBallLift to the trigger values
     *
     * gamepad2.left_trigger = raises capBallLift
     * gamepad2.right_trigger = lowers capBallLift
     */
    private void processCapBall(vv_OpMode anOpMode) throws InterruptedException {
        if(gamepad2.left_trigger > vv_Constants.TRIGGER_THRESHOLD) {
            vvLib.setCapBallLiftPower(anOpMode, (gamepad2.left_trigger * vv_Constants.CAP_BALL_SCORE_POWER_FACTOR));
        } else if (gamepad2.right_trigger > vv_Constants.TRIGGER_THRESHOLD) {
            vvLib.setCapBallLiftPower(anOpMode, (-gamepad2.right_trigger * vv_Constants.CAP_BALL_SCORE_POWER_FACTOR));
        } else {
            vvLib.setCapBallLiftPower(anOpMode, 0);
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
    private void processShootingAngle(vv_OpMode anOpMode) throws InterruptedException {
        //Changes Angle of the Shooting Mechanism to Position 1


       /* if (gamepad2.a) {
            vvLib.moveSpringMotorToPosition(anOpMode, vv_Constants.SpringPositionsEnum.Position1);
        }
        //Changes Angle of the Shooting Mechanism to Position 2
        if (gamepad2.x) {
            vvLib.moveSpringMotorToPosition(anOpMode, vv_Constants.SpringPositionsEnum.Position2);
        }
        //Changes Angle of the Shooting Mechanism to Position 3
        if (gamepad2.y) {
            vvLib.moveSpringMotorToPosition(anOpMode, vv_Constants.SpringPositionsEnum.Position3);
        }
        //Changes Angle of the Shooting Mechanism to Position 4
        if (gamepad2.b) {
            vvLib.moveSpringMotorToPosition(anOpMode, vv_Constants.SpringPositionsEnum.Position4);
        }
*/
        //TODO Change the Angle slowly using Joystick Y Values
    }

    /**
     * If the gamepad2.left_bumper is pressed x amount of times, the mechanism shoots the ball x
     * amount of times
     *
     * @param anOpMode object of vv_OpMode
     */
    private void processShooting(vv_OpMode anOpMode) throws InterruptedException {

        boolean isLeftBumperPressed;

        //Loads and Shoots one Ball
        if (gamepad2.left_bumper) {
            isLeftBumperPressed = true;
        } else {
            isLeftBumperPressed = false;
        }

        vvLib.shootOneBall(anOpMode, isLeftBumperPressed);
        //TODO: if we press the button x amount of times, shoot & load the ball x amount of times
    }
}
