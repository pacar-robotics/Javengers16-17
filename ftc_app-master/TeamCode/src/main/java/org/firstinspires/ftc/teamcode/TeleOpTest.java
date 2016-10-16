package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp(name = "TeleOpTest", group = "Test")

public class TeleOpTest extends vv_OpMode {

<<<<<<< HEAD
=======
	vv_Lib vvLib;
>>>>>>> origin/common-dev

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

        vvLib.moveTillTouch();
        while (opModeIsActive()) {

<<<<<<< HEAD
            /*if (Math.abs(gamepad1.right_stick_x) > vv_Constants.ANALOG_STICK_THRESHOLD) {
                robot.runMotors(gamepad1.right_stick_x, -gamepad1.right_stick_x, gamepad1.right_stick_x, -gamepad1.right_stick_x);
            }else if (Math.abs(gamepad1.left_stick_x) > vv_Constants.ANALOG_STICK_THRESHOLD ||
                    Math.abs(gamepad1.left_stick_y) > vv_Constants.ANALOG_STICK_THRESHOLD) {
                robot.runMotors((float)(gamepad1.left_stick_x*(Math.abs(Math.pow(gamepad1.left_stick_x,2.0)) - gamepad1.left_stick_y*(Math.abs(Math.pow(gamepad1.left_stick_y,2.0))))),
                        (float)(-gamepad1.left_stick_x*(Math.abs(Math.pow(gamepad1.left_stick_x,2.0)) - gamepad1.left_stick_y*(Math.abs(Math.pow(gamepad1.left_stick_y,2.0))))),
                        (float)(-gamepad1.left_stick_x*(Math.abs(Math.pow(gamepad1.left_stick_x,2.0)) - gamepad1.left_stick_y*(Math.abs(Math.pow(gamepad1.left_stick_y,2.0))))),
                        (float)(gamepad1.left_stick_x*(Math.abs(Math.pow(gamepad1.left_stick_x,2.0)) - gamepad1.left_stick_y*(Math.abs(Math.pow(gamepad1.left_stick_y,2.0))))));
            }else{

                robot.stopMotors();
            }*/


            if (gamepad1.x) {
=======
            if(gamepad1.x){
>>>>>>> origin/common-dev
                vvLib.pushAButton(vv_Constants.ButtonEnum.Left);
            } else if (gamepad1.b) {
                vvLib.pushAButton(vv_Constants.ButtonEnum.Right);
            }

            processDriveRobot();

            processBeacon();

            processCapBall();

<<<<<<< HEAD
=======
            processBallCollection();

            processShootingAngle();

            processShooting();

>>>>>>> origin/common-dev
            idle();
        }
    }

    private void processDriveRobot() throws InterruptedException
    {
        //            if(gamepad1.left_stick_x > vv_Constants.ANALOG_STICK_THRESHOLD) {
//                vvLib.moveSidewaysRight(robot, gamepad1.left_stick_x);
//            }else if(gamepad1.left_stick_x < -vv_Constants.ANALOG_STICK_THRESHOLD) {
//                vvLib.moveSidewaysLeft(robot, -gamepad1.left_stick_x);
//            }else if(gamepad1.left_stick_y > vv_Constants.ANALOG_STICK_THRESHOLD) {
//                vvLib.moveForward(robot, -gamepad1.left_stick_y);
//            }else if(gamepad1.left_stick_y < -vv_Constants.ANALOG_STICK_THRESHOLD) {
//                vvLib.moveBackward(robot, gamepad1.left_stick_y);
//            }else{
//                robot.stopMotors();
//            }
        if (Math.abs(gamepad1.right_stick_x) > vv_Constants.ANALOG_STICK_THRESHOLD)
        {
            vvLib.runAllMotors(this, gamepad1.right_stick_x, -gamepad1.right_stick_x, gamepad1.right_stick_x, -gamepad1.right_stick_x);
        }
        else if (Math.abs(gamepad1.left_stick_x) > vv_Constants.ANALOG_STICK_THRESHOLD ||
                Math.abs(gamepad1.left_stick_y) > vv_Constants.ANALOG_STICK_THRESHOLD)
        {
            vvLib.runAllMotors(this, ((Math.abs(gamepad1.left_stick_x) * gamepad1.left_stick_x) - ((Math.abs(gamepad1.left_stick_y)) * gamepad1.left_stick_y)),
                    (-(gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x)) -((Math.abs(gamepad1.left_stick_y) * gamepad1.left_stick_y))),
                    (-(gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x)) -((Math.abs(gamepad1.left_stick_y) * gamepad1.left_stick_y))),
                    ((Math.abs(gamepad1.left_stick_x) * gamepad1.left_stick_x) - ((Math.abs(gamepad1.left_stick_y)) * gamepad1.left_stick_y)));
        }else{
            vvLib.stopAllMotors();
        }
    }

    private void processBeacon() {
        // Changes Beacon Mechanism to left position in order to score the beacon
        if (gamepad1.dpad_left) {
        }
        // Changes Beacon Mechanism to right position in order to score the beacon
        if (gamepad1.dpad_right) {

        }
        // Changes Beacon Mechanism to a Neutral Position
        if (gamepad1.dpad_down) {

        }
    }

    private void processCapBall() {
        //TODO: if the lift is up, change drive factor to a small factor
        // Semi-Autonomously Raises the Cap Ball Lift Mechanism to the Scroing Height
        if (gamepad1.y) {

        }
        //Semi-Autonomously Lowers the Cap Ball Lift Mechanism to Resting Position
        if (gamepad1.a) {

        }
        //Grabs or Releases the Ball [Toggle]
        if (gamepad1.x) {

        }
    }

    private void processBallCollection() {
        //Changes state of Ball Collection mechanism to Outtake [Toggles On or Off]
        if (gamepad1.right_bumper){

        }
        //Changes state of Ball Collection mechanism to Outtake [Toggles On or Off]
        if (gamepad1.left_bumper){

        }
    }

    private void processShootingAngle() {
        //Changes Angle of the Shooting Mehcanism to Position 1
        if (gamepad2.a){

        }
        //Changes Angle of the Shooting Mehcanism to Position 2
        if (gamepad2.x){

        }
        //Changes Angle of the Shooting Mehcanism to Position 3
        if (gamepad2.y){

        }
        //Changes Angle of the Shooting Mehcanism to Position 4
        if (gamepad2.b){

        }

        //TODO Change the Angle slowly using Joystick Y Values
    }

    private void processShooting() {
        //Loads and Shoots one Ball
        if (gamepad2.left_bumper)
        {

        }
        //TODO: if we press the button x amount of times, shoot & load the ball x amount of times
    }
    public void telemetryAddData(String caption, String key, String message)
    {
        telemetry.addLine(caption).addData(key,message);
    }

    public void telemetryUpdate() {
        telemetry.update();
    }

}
