package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.vv_Constants.TRIGGER_THRESHOLD;


@TeleOp(name = "LauncherCalibrationOp", group = "Test")

public class LauncherCalibration extends vv_OpMode {

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


            processLaunchPowerCalibration();

            processLaunch();

            processDrive();

            processIntake();


            idle();

        }
    }


    public void processDrive()
            throws InterruptedException {
        vvLib.driveRobotWithPowerFactor(this, 0.4f);
    }


    /**
     * Changes the state of the Beacon Button Mechanism depending on the D-PAD Button Pressed
     * <p>
     * gamepad1.dpad_left = changes mechanism to left position gamepad1.dpad_right = changes
     * mechanism to right position gamepad1.dpad_down = changes mechanism to neutral position
     */
    private void processBeacon() {
        // Changes Beacon Mechanism to left position in order to score the beacon
        if (gamepad1.dpad_left) {
            vvLib.pushABeaconButton(this, vv_Constants.BeaconServoStateEnum.Left);
        }
        // Changes Beacon Mechanism to right position in order to score the beacon
        if (gamepad1.dpad_right) {
            vvLib.pushABeaconButton(this, vv_Constants.BeaconServoStateEnum.Right);
        }
        // Changes Beacon Mechanism to a Neutral Position
        if (gamepad1.dpad_down) {
            vvLib.pushABeaconButton(this, vv_Constants.BeaconServoStateEnum.Neutral);
        }
    }

    /**
     * Controls the Cap Ball Mechanism by either raising the lift, lowering the lift, or grabbing
     * the ball
     * <p>
     * gamepad1.y = raises the lift to scoring position gamepad1.a = lowers the lift to the resting
     * position gamepad.x = graps/releases the cap ball
     */
    private void processCapBall() throws InterruptedException {
        // Semi-Autonomously Raises the Cap Ball Lift Mechanism to the Scoring Height
            /*
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
            */
    }

    /**
     * Controls the Ball Collector Motor
     * <p>
     * gamepad1.right_bumper = turns the motor either to off or to the intake power
     * gamepad1.left_bumper = turns the motor either to off or to the outtake power
     */
    private void processIntake()
            throws InterruptedException {
        //Changes state of Ball Collection mechanism to Outtake [Toggles On or Off]
        if (gamepad1.right_bumper) {
            vvLib.toggleIntake(this);
        }
        //Changes state of Ball Collection mechanism to Outtake [Toggles On or Off]
        if (gamepad1.left_bumper) {
            vvLib.toggleOuttake(this);
        }
    }


    /**
     * Changes the power of the Shooting Mechanism
     * <p>
     * gamepad2.a = changes the mechanism to position 1 (___ Tiles away) gamepad2.x = changes the
     * mechanism to position 2 (___ Tiles away) gamepad2.y = changes the mechanism to position 3
     * (___ Tiles away) gamepad2.b = changes the mechanism to position 4 (___ Tiles away)
     */
    private void processLaunchPower() throws InterruptedException {
        //Changes Angle of the Shooting Mechanism to Position 1


       /* if (gamepad2.a) {
            vvLib.moveSpringMotorToPosition(aOpMode, vv_Constants.SpringPositionsEnum.Position1);
        }
        //Changes Angle of the Shooting Mechanism to Position 2
        if (gamepad2.x) {
            vvLib.moveSpringMotorToPosition(aOpMode, vv_Constants.SpringPositionsEnum.Position2);
        }
        //Changes Angle of the Shooting Mechanism to Position 3
        if (gamepad2.y) {
            vvLib.moveSpringMotorToPosition(aOpMode, vv_Constants.SpringPositionsEnum.Position3);
        }
        //Changes Angle of the Shooting Mechanism to Position 4
        if (gamepad2.b) {
            vvLib.moveSpringMotorToPosition(aOpMode, vv_Constants.SpringPositionsEnum.Position4);
        }
*/
        //TODO Change the Angle slowly using Joystick Y Values
    }

    /**
     * If the gamepad2.left_bumper is pressed x amount of times, the mechanism shoots the ball x
     * amount of times
     */
    private void processLaunch()
            throws InterruptedException {

/*
        if (gamepad1.y) {
            //shoot a ball
            //then setup for next shot.
            //first setupShot should be called from vvLib.init


            try {
                //first move to position max
                vvLib.setLauncherPowerPosition(this, LAUNCH_POWER_POSITION_MAX);

            } catch (vv_Robot.MotorStalledException MSE) {
                telemetryAddData("Motor Stalled!", "Name", MSE.getMessage());
                Thread.sleep(500);
            }
            if (vvLib.getLauncherPowerPosition(this) > (LAUNCH_POWER_POSITION_MAX - LAUNCH_POWER_INCREMENT)) {
                vvLib.shootBall(this);
                vvLib.setupShot(this);
            }
        }

*/

        /*

        if (gamepad1.x) {
            //shoot a ball
            //then setup for next shot.
            //first setupShot should be called from vvLib.init


            //first move to position max
            vvLib.setLauncherPowerPosition(this, LAUNCH_POWER_POSITION_MID);

            vvLib.shootBall(this);
            vvLib.setupShot(this);
        }

        if (gamepad1.a) {
            //shoot a ball
            //then setup for next shot.
            //first setupShot should be called from vvLib.init


            //first move to position max
            vvLib.setLauncherPowerPosition(this, LAUNCH_POWER_POSITION_AUTONOMOUS);

            vvLib.shootBall(this);
            vvLib.setupShot(this);
        }

*/
        if (gamepad1.start) {
            //shoot a ball
            //then setup for next shot.
            //launch where we are.
            //used to calibrate the location.


            vvLib.shootBall(this);
            vvLib.setupShot(this);
        }


/*
        if (gamepad1.b) {
            //shoot a ball
            //then setup for next shot.
            //first setupShot should be called from vvLib.init


            //first move to position max
            vvLib.setLauncherPowerPosition(this, LAUNCH_POWER_POSITION_MIN);
        }

        */


    }

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
            Thread.sleep(500);
        }
    }
}