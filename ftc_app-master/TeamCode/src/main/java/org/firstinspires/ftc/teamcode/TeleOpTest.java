package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static java.lang.Math.pow;


@TeleOp(name="TeleOpTest", group="Test")

public class TeleOpTest extends LinearOpMode {

    /* Declare OpMode members. */
    vv_Robot robot           = new vv_Robot();


    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        vv_Lib vvLib = new vv_Lib();


        while(opModeIsActive()) {
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
            if (Math.abs(gamepad1.right_stick_x) > vv_Constants.ANALOG_STICK_THRESHOLD) {
                robot.runMotors(gamepad1.right_stick_x, -gamepad1.right_stick_x, gamepad1.right_stick_x, -gamepad1.right_stick_x);
            }else if (Math.abs(gamepad1.left_stick_x) > vv_Constants.ANALOG_STICK_THRESHOLD ||
                    Math.abs(gamepad1.left_stick_y) > vv_Constants.ANALOG_STICK_THRESHOLD) {
                robot.runMotors((float)((Math.pow(gamepad1.left_stick_x,2.0) - (Math.pow(gamepad1.left_stick_y,2.0)))),
                        (float)(-(Math.pow(gamepad1.left_stick_x,2.0) - (Math.pow(gamepad1.left_stick_y,2.0)))),
                        (float)(-(Math.pow(gamepad1.left_stick_x,2.0) - (Math.pow(gamepad1.left_stick_y,2.0)))),
                        (float)((Math.pow(gamepad1.left_stick_x,2.0) - (Math.pow(gamepad1.left_stick_y,2.0)))));
            }else{
                robot.stopMotors();
            }



            idle();
        }
    }

    public void telemetryAddData(String caption, String key, String message){
        telemetry.addLine(caption).addData(key,message);
    }
    public void telemetryUpdate() {
        telemetry.update();
    }

}
