package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp(name = "TeleOpTest", group = "Test")

public class TeleOpTest extends vv_OpMode {


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


        vv_Lib vvLib = new vv_Lib(this);

        vvLib.moveTillTouch();
        while (opModeIsActive()) {

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
                vvLib.pushAButton(vv_Constants.ButtonEnum.Left);
            } else if (gamepad1.b) {
                vvLib.pushAButton(vv_Constants.ButtonEnum.Right);
            }


            idle();
        }
    }
}
