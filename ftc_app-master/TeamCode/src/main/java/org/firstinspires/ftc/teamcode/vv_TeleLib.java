package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.vv_Constants.CAP_BALL_ENCODER_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.vv_Constants.CAP_BALL_POSITION_INCREMENT;
import static org.firstinspires.ftc.teamcode.vv_Constants.EOPD_PROXIMITY_THRESHOLD;
import static org.firstinspires.ftc.teamcode.vv_Constants.INTAKE_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.LAUNCH_POSITION_INCREMENT;
import static org.firstinspires.ftc.teamcode.vv_Constants.TRIGGER_THRESHOLD;
import static org.firstinspires.ftc.teamcode.vv_Constants.GENERIC_TIMER;
import static org.firstinspires.ftc.teamcode.vv_Constants.DPAD_TIMER;

/**
 * Created by thomas on 9/25/2016.
 */

public class vv_TeleLib {



    protected void processIntake(vv_OpMode aOpMode, vv_Lib vvLib) throws InterruptedException {
        //Changes state of Ball Collection mechanism to Outtake [Toggles On or Off]
        if (aOpMode.gamepad1.right_bumper) {
            vvLib.toggleIntake(aOpMode);
            Thread.sleep(250);
        }
        //Changes state of Ball Collection mechanism to Outtake [Toggles On or Off]
        if (aOpMode.gamepad1.left_bumper) {
            vvLib.toggleOuttake(aOpMode);
            Thread.sleep(250);
        }
    }

    protected void processParticleBallLaunch(vv_OpMode aOpMode, vv_Lib vvLib) throws InterruptedException {
        if ((aOpMode.gamepad1.a)&&(!aOpMode.gamepad1.start)) {
            //shoot a ball
            //then setup for next shot.
            //launch where we are.
            //used to calibrate the location.

            vvLib.shootBallAndSpinIntake(aOpMode);
            //absorb any extra button presses
            Thread.sleep(150);
        }
        if ((aOpMode.gamepad1.b)&&(!aOpMode.gamepad1.start)) {
            //drop the ball but do not shoot.
            vvLib.dropBall(aOpMode);
            Thread.sleep(150);
        }
        if (aOpMode.gamepad1.x) {
            //dont drop the ball but shoot whats there
            vvLib.shootBall(aOpMode);
            vvLib.setupShot(aOpMode);
            Thread.sleep(150);
        }
    }


    protected void processBallFlag(vv_OpMode aOpMode, vv_Lib vvLib) throws InterruptedException {
        //detect if there is a ball in the intake ready to be dropped into
        //launcher.
        if (vvLib.getEopdRawValue(aOpMode) > EOPD_PROXIMITY_THRESHOLD) {
            vvLib.raiseBallFlagServo(aOpMode);
        } else {
            vvLib.lowerBallFlagServo(aOpMode);
        }

    }


    protected void processLaunchPowerCalibration(vv_OpMode aOpMode, vv_Lib vvLib) throws InterruptedException {

        try {
            aOpMode.telemetry.setAutoClear(true);
            aOpMode.telemetryAddData("Power Position Before:", "Value", ":" +
                    vvLib.getLauncherPowerPosition(aOpMode));
            aOpMode.telemetryUpdate();

            //read the triggers from game pad.
            if (aOpMode.gamepad1.left_trigger > TRIGGER_THRESHOLD) {
                vvLib.decreaseLauncherPower(aOpMode);
            }
            if (aOpMode.gamepad1.right_trigger > TRIGGER_THRESHOLD) {
                vvLib.increaseLauncherPower(aOpMode);
            }
            aOpMode.telemetryAddData("Power Position After:", "Value", ":" +
                    vvLib.getLauncherPowerPosition(aOpMode));
            aOpMode.telemetryUpdate();
        } catch (vv_Robot.MotorStalledException MSE) {
            aOpMode.telemetryAddData("Motor Stalled!", "Name", MSE.getMessage());
            aOpMode.telemetryUpdate();
            Thread.sleep(50);
        }
    }

    public void processNonFieldOrientedDrive(vv_OpMode aOpMode, vv_Lib vvLib, float powerFactor)
            throws InterruptedException {
//non field oriented
        //not used in current teleops. Here for legacy purposes and fall back if needed.


        float forwardLeftPower = 0;
        float backLeftPower = 0;
        float backRightPower = 0;
        float forwardRightPower = 0;

        if (aOpMode.gamepad1.left_stick_y > vv_Constants.ANALOG_STICK_THRESHOLD &&
                Math.abs(aOpMode.gamepad1.left_stick_x) < vv_Constants.ANALOG_STICK_THRESHOLD) {
            forwardLeftPower = aOpMode.gamepad1.left_stick_y;
            backLeftPower = -aOpMode.gamepad1.left_stick_y;
            backRightPower = aOpMode.gamepad1.left_stick_y;
            forwardRightPower = -aOpMode.gamepad1.left_stick_y;
        } else if (aOpMode.gamepad1.left_stick_y < -vv_Constants.ANALOG_STICK_THRESHOLD &&
                Math.abs(aOpMode.gamepad1.left_stick_x) < vv_Constants.ANALOG_STICK_THRESHOLD) {
            forwardLeftPower = aOpMode.gamepad1.left_stick_y;
            backLeftPower = -aOpMode.gamepad1.left_stick_y;
            backRightPower = aOpMode.gamepad1.left_stick_y;
            forwardRightPower = -aOpMode.gamepad1.left_stick_y;
        } else {
            forwardLeftPower = aOpMode.gamepad1.left_stick_x;
            backLeftPower = aOpMode.gamepad1.left_stick_x;
            backRightPower = aOpMode.gamepad1.left_stick_x;
            forwardRightPower = aOpMode.gamepad1.left_stick_x;
        }


        //rotates or turns the robot
        if (Math.abs(aOpMode.gamepad1.right_stick_x) > vv_Constants.ANALOG_STICK_THRESHOLD) {
            vvLib.runAllMotors(aOpMode, (aOpMode.gamepad1.right_stick_x * powerFactor), (-aOpMode.gamepad1.right_stick_x * powerFactor),
                    (aOpMode.gamepad1.right_stick_x * powerFactor), (-aOpMode.gamepad1.right_stick_x * powerFactor));
        }
        //translates the robot using the Mecanum wheels
        else if (Math.abs(aOpMode.gamepad1.left_stick_x) > vv_Constants.ANALOG_STICK_THRESHOLD ||
                Math.abs(aOpMode.gamepad1.left_stick_y) > vv_Constants.ANALOG_STICK_THRESHOLD) {
            vvLib.runAllMotors(aOpMode, (forwardLeftPower * powerFactor), (forwardRightPower * powerFactor), (backLeftPower * powerFactor), (backRightPower * powerFactor));
        } else {
            vvLib.stopAllMotors(aOpMode);
        }
    }


    public void processFieldOrientedDrive(vv_OpMode aOpMode, vv_Lib vvLib, float powerFactor)
            throws InterruptedException {

        //process joysticks

        if (Math.abs(aOpMode.gamepad1.left_stick_x) > vv_Constants.ANALOG_STICK_THRESHOLD ||
                Math.abs(aOpMode.gamepad1.left_stick_y) > vv_Constants.ANALOG_STICK_THRESHOLD) {
            //we are not in deadzone. Driver is pushing left joystick
            //lets make the robot move in chosen angle and magnitude.
            vvLib.universalMoveRobotForFieldOrientedTeleOp(aOpMode,
                    vvLib.robot.getGamePad1LeftJoystickPolarMagnitude(aOpMode) * powerFactor,
                    vvLib.robot.getGamePad1LeftJoystickPolarAngle(aOpMode)
                            + 90 - //for rotated orientation of robot at start of game.
                            vvLib.robot.getMxpGyroSensorHeading(aOpMode)); //for yaw on field.


        }
        if (Math.abs(aOpMode.gamepad1.right_stick_x) > vv_Constants.ANALOG_STICK_THRESHOLD) {

            //we are not in deadzone. Driver is pushing right joystick, sideways
            float turnVelocity = (float) vvLib.robot.getGamePad1RightJoystickPolarMagnitude(aOpMode) * powerFactor;

            if (aOpMode.gamepad1.right_stick_x > 0) {
                //turn clockwise to correct magnitude
                vvLib.robot.runMotors(aOpMode, turnVelocity, -turnVelocity, turnVelocity, -turnVelocity);
            } else {
                //turn counter-clockwise
                vvLib.robot.runMotors(aOpMode, -turnVelocity, turnVelocity, -turnVelocity, turnVelocity);
            }


        }

        if ((Math.abs(aOpMode.gamepad1.left_stick_x) < vv_Constants.ANALOG_STICK_THRESHOLD &&
                Math.abs(aOpMode.gamepad1.left_stick_y) < vv_Constants.ANALOG_STICK_THRESHOLD) &&
                Math.abs(aOpMode.gamepad1.right_stick_x) < vv_Constants.ANALOG_STICK_THRESHOLD &&
                (Math.abs(aOpMode.gamepad2.left_stick_x) < vv_Constants.ANALOG_STICK_THRESHOLD &&
                        Math.abs(aOpMode.gamepad2.left_stick_y) < vv_Constants.ANALOG_STICK_THRESHOLD) &&
                Math.abs(aOpMode.gamepad2.right_stick_x) < vv_Constants.ANALOG_STICK_THRESHOLD && !aOpMode.gamepad1.dpad_down
            && !aOpMode.gamepad1.dpad_left && !aOpMode.gamepad1.dpad_up && !aOpMode.gamepad2.dpad_right)
        {
            //both joysticks on both gamepads are at rest, stop the robot.

            vvLib.stopAllMotors(aOpMode);
        }


    }

    public void processFieldOrientedCapBallDrive(vv_OpMode aOpMode, vv_Lib vvLib, float powerFactor)
            throws InterruptedException, vv_Robot.MotorStalledException {

        //process joysticks

        if (Math.abs(aOpMode.gamepad2.left_stick_x) > vv_Constants.ANALOG_STICK_THRESHOLD ||
                Math.abs(aOpMode.gamepad2.left_stick_y) > vv_Constants.ANALOG_STICK_THRESHOLD) {
            //we are not in deadzone. Driver is pushing left joystick
            //lets make the robot move in chosen angle and magnitude.

            vvLib.universalMoveRobotForFieldOrientedTeleOp(aOpMode,
                    vvLib.robot.getGamePad2LeftJoystickPolarMagnitude(aOpMode) * powerFactor,
                    vvLib.robot.getGamePad2LeftJoystickPolarAngle(aOpMode)
                            + 90 - //for rotated orientation of robot at start of game.
                            vvLib.robot.getMxpGyroSensorHeading(aOpMode)); //for yaw on field.


        }
        if (Math.abs(aOpMode.gamepad2.right_stick_x) > vv_Constants.ANALOG_STICK_THRESHOLD) {

            //we are not in deadzone. Driver is pushing right joystick, sideways
            //scale the powerfactor by another factor to adjust for observed slow turns.
            float turnVelocity = (float) vvLib.robot.getGamePad2RightJoystickPolarMagnitude(aOpMode) * powerFactor*1.5f;

            if (aOpMode.gamepad2.right_stick_x > 0) {
                //turn clockwise to correct magnitude
                vvLib.robot.runMotors(aOpMode, turnVelocity, -turnVelocity, turnVelocity, -turnVelocity);
            } else {
                //turn counter-clockwise
                vvLib.robot.runMotors(aOpMode, -turnVelocity, turnVelocity, -turnVelocity, turnVelocity);
            }


        }

        if ((Math.abs(aOpMode.gamepad1.left_stick_x) < vv_Constants.ANALOG_STICK_THRESHOLD &&
                Math.abs(aOpMode.gamepad1.left_stick_y) < vv_Constants.ANALOG_STICK_THRESHOLD) &&
                Math.abs(aOpMode.gamepad1.right_stick_x) < vv_Constants.ANALOG_STICK_THRESHOLD &&
                (Math.abs(aOpMode.gamepad2.left_stick_x) < vv_Constants.ANALOG_STICK_THRESHOLD &&
                        Math.abs(aOpMode.gamepad2.left_stick_y) < vv_Constants.ANALOG_STICK_THRESHOLD) &&
                Math.abs(aOpMode.gamepad2.right_stick_x) < vv_Constants.ANALOG_STICK_THRESHOLD&& !aOpMode.gamepad1.dpad_down
                && !aOpMode.gamepad1.dpad_left && !aOpMode.gamepad1.dpad_up && !aOpMode.gamepad2.dpad_right) {
            //both joysticks on both gamepads are at rest, stop the robot.

            vvLib.stopAllMotors(aOpMode);
        }


    }


    protected void processBeaconOrientationControls(vv_OpMode aOpMode, vv_Lib vvLib) throws InterruptedException {
        //process dpads
        if (aOpMode.gamepad1.dpad_down) {
                vvLib.turnAbsoluteMxpGyroDegrees(aOpMode, 0);
        }

        if (aOpMode.gamepad1.dpad_up) {
                vvLib.turnAbsoluteMxpGyroDegrees(aOpMode, +180);
        }
        if (aOpMode.gamepad1.dpad_right) {
            vvLib.turnAbsoluteMxpGyroDegrees(aOpMode, -90);
        }
        if (aOpMode.gamepad1.dpad_left) {
            vvLib.turnAbsoluteMxpGyroDegrees(aOpMode, +90);
        }
    }


    protected void processYawReset(vv_OpMode aOpMode, vv_Lib vvLib) throws InterruptedException {

        //process yaw reset

        if (aOpMode.gamepad1.y) {
            //reset yaw, if the right trigger is not also pressed.
            //if the right trigger is also pressed, this is a command to reset the
            // choo choo encoder
            vvLib.robot.setMxpGyroZeroYaw(aOpMode);
        }


    }


    public void processCapBallControls(vv_OpMode aOpMode, vv_Lib vvLib, vv_TeleLib vvTeleLib) throws InterruptedException,
            vv_Robot.MotorStalledException {

        if (aOpMode.gamepad2.right_bumper) {
            vvLib.robot.releaseCapbBallHolder(aOpMode);
        }

        if (aOpMode.gamepad2.left_bumper) {
            vvLib.robot.secureCapbBallHolder(aOpMode);
        }

        if (aOpMode.gamepad2.left_trigger > TRIGGER_THRESHOLD) {
            //we want to move cap ball to go down
            vvLib.robot.releaseCapbBallHolder(aOpMode);
            if (vvLib.robot.getCapBallMotorEncoderPosition(aOpMode) <= 0) {
                //at limit
                //warn and set the robot to min
                aOpMode.telemetryAddData("WARNING", "CAP BALL LIMIT:", "at " +
                        vvLib.robot.getCapBallMotorEncoderPosition(aOpMode));
                lowerCapBallToMinHeight(aOpMode, vvLib, vvTeleLib);
            } else {
                if (vvLib.robot.getCapBallMotorEncoderPosition(aOpMode) <= CAP_BALL_POSITION_INCREMENT) {
                    //not enough space left to go a full increment.
                    lowerCapBallToMinHeight(aOpMode, vvLib, vvTeleLib);
                } else {
                    decreaseCapBallHeight(aOpMode, vvLib,
                            CAP_BALL_POSITION_INCREMENT, vvTeleLib);
                }
            }
        }

        if (aOpMode.gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            //we want to move cap ball to go up
            vvLib.robot.releaseCapbBallHolder(aOpMode);
            if (vvLib.robot.getCapBallMotorEncoderPosition(aOpMode) >= CAP_BALL_ENCODER_UPPER_LIMIT) {
                //at limit
                //do nothing but warn on screen.
                aOpMode.telemetryAddData("WARNING", "CAP BALL LIMIT:", "at " +
                        vvLib.robot.getCapBallMotorEncoderPosition(aOpMode));
                raiseCapBallToMaxHeight(aOpMode, vvLib, vvTeleLib);
            } else {
                if (vvLib.robot.getCapBallMotorEncoderPosition(aOpMode) >=
                        (CAP_BALL_ENCODER_UPPER_LIMIT - CAP_BALL_POSITION_INCREMENT)) {
                    //not enough space left to go a full increment.
                    raiseCapBallToMaxHeight(aOpMode, vvLib, vvTeleLib);
                } else {

                    increaseCapBallHeight(aOpMode, vvLib,
                           CAP_BALL_POSITION_INCREMENT);
                }
            }
        }

        if (aOpMode.gamepad2.a) {
            vvLib.robot.releaseCapbBallHolder(aOpMode);
            lowerCapBallToMinHeight(aOpMode, vvLib, vvTeleLib);
        }

        if (aOpMode.gamepad2.x) {
            scoreCapBall(aOpMode, vvLib);
        }

        if (aOpMode.gamepad2.y) {
            vvLib.robot.releaseCapbBallHolder(aOpMode);
            raiseCapBallToMaxHeight(aOpMode, vvLib, vvTeleLib);
        }

    }

    public void processCapBallControlsWithoutLimits(vv_OpMode aOpMode, vv_Lib vvLib, vv_TeleLib vvTeleLib) throws InterruptedException,
            vv_Robot.MotorStalledException {

        //be very careful, the cable may break.

        if (aOpMode.gamepad2.left_trigger > TRIGGER_THRESHOLD) {
            vvLib.robot.releaseCapbBallHolder(aOpMode);
            decreaseCapBallHeight(aOpMode, vvLib,
                    CAP_BALL_POSITION_INCREMENT, vvTeleLib);
        }

        if (aOpMode.gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            vvLib.robot.releaseCapbBallHolder(aOpMode);
            increaseCapBallHeight(aOpMode, vvLib,
                     CAP_BALL_POSITION_INCREMENT);
        }

        if (aOpMode.gamepad2.right_bumper) {
            vvLib.robot.releaseCapbBallHolder(aOpMode);
        }

        if (aOpMode.gamepad2.left_bumper) {
            vvLib.robot.secureCapbBallHolder(aOpMode);
        }


        aOpMode.telemetryAddData("Cap Ball Height:", "Encoder:", "Value:" +
                vvLib.robot.getCapBallMotorEncoderPosition(aOpMode));
        aOpMode.telemetryUpdate();

    }

    public void processCapBallControlsWithoutStall(vv_OpMode aOpMode, vv_Lib vvLib, vv_TeleLib vvTeleLib) throws InterruptedException,
            vv_Robot.MotorStalledException {

        //be very careful, the cable may break.

        if (aOpMode.gamepad2.left_trigger > TRIGGER_THRESHOLD) {
            decreaseCapBallHeightNoStall(aOpMode, vvLib,
                     CAP_BALL_POSITION_INCREMENT);
        }

        if (aOpMode.gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            increaseCapBallHeightNoStall(aOpMode, vvLib,
                   CAP_BALL_POSITION_INCREMENT, vvTeleLib);
        }
        aOpMode.telemetryAddData("Cap Ball Height:", "Encoder:", "Value:" +
                vvLib.robot.getCapBallMotorEncoderPosition(aOpMode));
        aOpMode.telemetryUpdate();

    }

    public void raiseCapBallToMaxHeight(vv_OpMode aOpMode, vv_Lib vvLib, vv_TeleLib vvTeleLib) throws InterruptedException,
            vv_Robot.MotorStalledException {
        vvLib.robot.setCapBallPosition(aOpMode, Math.round(CAP_BALL_ENCODER_UPPER_LIMIT), vvLib, vvTeleLib);
    }

    public void lowerCapBallToMinHeight(vv_OpMode aOpMode, vv_Lib vvLib, vv_TeleLib vvTeleLib) throws InterruptedException,
            vv_Robot.MotorStalledException {
        vvLib.robot.setCapBallPosition(aOpMode, 0, vvLib, vvTeleLib);
    }

    public void setCapBallReleaseServoPosition(vv_OpMode aOpMode, vv_Lib vvLib, double servoPosition) {
        vvLib.robot.setCapBallReleaseServoPosition(aOpMode, servoPosition);
    }

    public double getCapBallReleaseServoPosition(vv_OpMode aOpMode, vv_Lib vvLib) {
        return vvLib.robot.getCapBallReleaseServoPosition(aOpMode);
    }

    public void decreaseCapBallHeight(vv_OpMode aOpMode, vv_Lib vvLib, int increment, vv_TeleLib vvTeleLib) throws InterruptedException,
            vv_Robot.MotorStalledException {
        vvLib.robot.setCapBallPosition(aOpMode, vvLib.robot.getCapBallMotorEncoderPosition(aOpMode) - increment, vvLib, vvTeleLib);
    }

    public void increaseCapBallHeightNoStall(vv_OpMode aOpMode, vv_Lib vvLib, int increment, vv_TeleLib vvTeleLib) throws InterruptedException,
            vv_Robot.MotorStalledException {
        vvLib.robot.setCapBallPosition(aOpMode, vvLib.robot.getCapBallMotorEncoderPosition(aOpMode) + increment, vvLib, vvTeleLib);
    }

    public void decreaseCapBallHeightNoStall(vv_OpMode aOpMode, vv_Lib vvLib, int increment) throws InterruptedException,
            vv_Robot.MotorStalledException {
        vvLib.robot.setCapBallPositionNoStall(aOpMode, vvLib.robot.getCapBallMotorEncoderPosition(aOpMode) - increment);
    }

    public void increaseCapBallHeight(vv_OpMode aOpMode, vv_Lib vvLib, int increment) throws InterruptedException,
            vv_Robot.MotorStalledException {
        vvLib.robot.setCapBallPositionNoStall(aOpMode, vvLib.robot.getCapBallMotorEncoderPosition(aOpMode) + increment);
    }


    public int getCapBallPosition(vv_OpMode aOpMode, vv_Lib vvLib) {
        return vvLib.robot.getCapBallMotorEncoderPosition(aOpMode);
    }

    public void scoreCapBall(vv_OpMode aOpMode, vv_Lib vvLib) throws InterruptedException{
        //move forward gently
        vvLib.moveWheels(aOpMode, 6, 0.5f, vv_Constants.DirectionEnum.Forward, true);
        //pull back suddenly
        vvLib.moveWheels(aOpMode, 4, 0.9f, vv_Constants.DirectionEnum.Backward, false);
    }



    protected void processChooChooPosition(vv_OpMode aOpMode, vv_Lib vvLib) throws InterruptedException {

        if (aOpMode.gamepad1.left_stick_button) {
            vvLib.robot.rotateChooChoo(aOpMode, LAUNCH_POSITION_INCREMENT);
            Thread.sleep(100); // to absorb extra button presses
        }
        if (aOpMode.gamepad1.right_stick_button) {
            vvLib.robot.rotateChooChoo(aOpMode, -LAUNCH_POSITION_INCREMENT);
            Thread.sleep(100); // to absorb extra button presses
        }
        if (aOpMode.gamepad1.start) {
            //reset our encoder for the choo choo arm at this position.
            //so launch will restart from here.

            vvLib.robot.resetChooChooEncoder(aOpMode);
        }

    }

//    public boolean isButtonLongPressed(vv_OpMode anOp, vv_Lib vvLib, boolean button_press) throws InterruptedException {
//
//    }
}

