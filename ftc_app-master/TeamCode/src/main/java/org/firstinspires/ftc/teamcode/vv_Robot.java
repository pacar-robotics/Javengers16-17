package org.firstinspires.ftc.teamcode;


import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;


/**
 * Created by thomas on 9/25/2016.
 */

public class vv_Robot {
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor armMotor = null;

    private Servo buttonServo = null;

    private TouchSensor buttonSensor;
    private TouchSensor armSensor;
    private ColorSensor cs;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();


    public void init(HardwareMap ahwMap, vv_OpMode aOpMode) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftMotor = hwMap.dcMotor.get("motor_front_left");
        frontRightMotor = hwMap.dcMotor.get("motor_front_right");
        backLeftMotor = hwMap.dcMotor.get("motor_back_left");
        backRightMotor = hwMap.dcMotor.get("motor_back_right");
        armMotor = hwMap.dcMotor.get("motor_arm");

        cs = hwMap.colorSensor.get("color_line_sensor");

        armSensor = hwMap.touchSensor.get("touch_arm_sensor");

        buttonServo = hwMap.servo.get("button_servo");

        buttonSensor = hwMap.touchSensor.get("touch_button_sensor");

        buttonServo.setPosition(0.65);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to zero power
        stopMotors(aOpMode);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

    }

    public void setPower(vv_OpMode aOpMode, vv_Constants.MotorEnum motorEnum, float power) {
        switch (motorEnum) {
            case frontLeftMotor:
                frontLeftMotor.setPower(power);
                break;
            case frontRightMotor:
                frontRightMotor.setPower(power);
                break;
            case backLeftMotor:
                backLeftMotor.setPower(power);
                break;
            case backRightMotor:
                backRightMotor.setPower(power);
                break;
            case armMotor:
                armMotor.setPower(power);
                break;
        }
    }

    public void setMotorMode(vv_OpMode aOpMode, vv_Constants.MotorEnum motorEnum, DcMotor.RunMode runMode) {
        if (motorEnum.equals("armMotor")) {
            armMotor.setMode(runMode);
        }
        //TODO: Finish this emthod up
    }

    public boolean isArmAtLimit(vv_OpMode aOpMode) {
        return armSensor.isPressed();
        //TODO: Finish this method up
    }


    public void runRobotToPositionFB(vv_OpMode aOpMode, int position, float Power) throws InterruptedException {
        //using the generic method with all powers set to the same value and all positions set to the same position
        runRobotToPosition(aOpMode, Power, Power, Power, Power, position, position, position, position);
    }

    /**
     * Runs robot to a specific position while driving sideways.
     *
     * @param aOpMode  an object of the vv_OpMode class
     * @param position generic position of the motors
     * @param Power    generic power of the motors
     * @return void
     */
    public void runRobotToPositionSideways(vv_OpMode aOpMode, int position, float Power) throws InterruptedException {
        //using the generic method with all powers set to the same value and all positions set to the same position
        runRobotToPosition(aOpMode, -Power, Power, Power, -Power, -position, position, position, -position);
    }


    /**
     * Runs robot to a specific position. Can be called by other, more specific methods to move forwards and backwards or sideways.
     *
     * @param aOpMode     an object of the vv_OpMode class
     * @param fl_Power    front right motor power
     * @param fr_Power    front left motor power
     * @param bl_Power    back left motor power
     * @param br_Power    back right motor power
     * @param fl_Position front left motor position
     * @param fr_Position front left motor position
     * @param bl_Position back left motor position
     * @param br_Position back right motor position
     * @return void
     */
    public void runRobotToPosition(vv_OpMode aOpMode, float fl_Power, float fr_Power,
                                   float bl_Power, float br_Power, int fl_Position,
                                   int fr_Position, int bl_Position, int br_Position)
            throws InterruptedException {
        //reset motor encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (frontLeftMotor.getCurrentPosition() != 0) {
            //wait until motors are reset
            Thread.sleep(20);
        }

        //sets all motors to run to a position
        setMotorMode(aOpMode, vv_Constants.MotorEnum.frontLeftMotor, DcMotor.RunMode.RUN_TO_POSITION);
        setMotorMode(aOpMode, vv_Constants.MotorEnum.frontRightMotor, DcMotor.RunMode.RUN_TO_POSITION);
        setMotorMode(aOpMode, vv_Constants.MotorEnum.backLeftMotor, DcMotor.RunMode.RUN_TO_POSITION);
        setMotorMode(aOpMode, vv_Constants.MotorEnum.backRightMotor, DcMotor.RunMode.RUN_TO_POSITION);

        //reset encoder for 1 wheel
        frontLeftMotor.setTargetPosition(fl_Position);
        frontRightMotor.setTargetPosition(fr_Position);
        backLeftMotor.setTargetPosition(bl_Position);
        backRightMotor.setTargetPosition(br_Position);

        //sets the the power of all motors
        setPower(aOpMode, vv_Constants.MotorEnum.frontLeftMotor, fl_Power);
        setPower(aOpMode, vv_Constants.MotorEnum.frontRightMotor, fr_Power);
        setPower(aOpMode, vv_Constants.MotorEnum.backLeftMotor, bl_Power);
        setPower(aOpMode, vv_Constants.MotorEnum.backRightMotor, br_Power);

        //wait until robot reaches target position
        //testing the wheels on the opposite sides of the robot because each might have a different position for sideways movements

        while ((Math.abs(frontLeftMotor.getCurrentPosition()) < Math.abs(fl_Position) - vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN) ||
                (Math.abs(frontRightMotor.getCurrentPosition()) < Math.abs(fr_Position) - vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN)
                || (Math.abs(backRightMotor.getCurrentPosition()) < Math.abs(br_Position) - vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN) ||
                (Math.abs(backLeftMotor.getCurrentPosition()) < Math.abs(bl_Position) - vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN)) {
            //report motor positions for debugging

            // TODO: UNCOMMENT THIS!!!!
            aOpMode.telemetryAddData("Motor FL", "Values", "" + frontLeftMotor.getCurrentPosition());
            aOpMode.telemetryAddData("Motor FR", "Values", "" + frontRightMotor.getCurrentPosition());
            aOpMode.telemetryAddData("Motor BL", "Values", "" + backLeftMotor.getCurrentPosition());
            aOpMode.telemetryAddData("Motor BR", "Values", "" + backRightMotor.getCurrentPosition());
            aOpMode.telemetryUpdate();

        }
        stopMotors(aOpMode);

        Thread.sleep(100);
    }


    public void runRobotToPositionWithAngle(vv_OpMode aOpMode, float fl_Power, float fr_Power,
                                            float bl_Power, float br_Power, int fl_Position,
                                            int fr_Position, int bl_Position, int br_Position, float angle)
            throws InterruptedException {

        //reset motor encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (frontLeftMotor.getCurrentPosition() != 0) {
            //wait until motors are reset
            Thread.sleep(20);
        }

        //sets all motors to run to a position
        setMotorMode(aOpMode, vv_Constants.MotorEnum.frontLeftMotor, DcMotor.RunMode.RUN_TO_POSITION);
        setMotorMode(aOpMode, vv_Constants.MotorEnum.frontRightMotor, DcMotor.RunMode.RUN_TO_POSITION);
        setMotorMode(aOpMode, vv_Constants.MotorEnum.backLeftMotor, DcMotor.RunMode.RUN_TO_POSITION);
        setMotorMode(aOpMode, vv_Constants.MotorEnum.backRightMotor, DcMotor.RunMode.RUN_TO_POSITION);

        //reset encoder for 1 wheel
        frontLeftMotor.setTargetPosition(fl_Position);
        frontRightMotor.setTargetPosition(fr_Position);
        backLeftMotor.setTargetPosition(bl_Position);
        backRightMotor.setTargetPosition(br_Position);

        //sets the the power of all motors
        setPower(aOpMode, vv_Constants.MotorEnum.frontLeftMotor, fl_Power);
        setPower(aOpMode, vv_Constants.MotorEnum.frontRightMotor, fr_Power);
        setPower(aOpMode, vv_Constants.MotorEnum.backLeftMotor, bl_Power);
        setPower(aOpMode, vv_Constants.MotorEnum.backRightMotor, br_Power);

        //wait until robot reaches target position
        //testing the wheels on the opposite sides of the robot because each might have a different position for sideways movements

        while ((Math.abs(frontLeftMotor.getCurrentPosition()) < Math.abs(fl_Position) - vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN) ||
                (Math.abs(frontRightMotor.getCurrentPosition()) < Math.abs(fr_Position) - vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN)
                || (Math.abs(backRightMotor.getCurrentPosition()) < Math.abs(br_Position) - vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN) ||
                (Math.abs(backLeftMotor.getCurrentPosition()) < Math.abs(bl_Position) - vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN)) {
            //report motor positions for debugging
            aOpMode.telemetryAddData("Motor FL", "Values", "" + frontLeftMotor.getCurrentPosition());
            aOpMode.telemetryAddData("Motor FR", "Values", "" + frontRightMotor.getCurrentPosition());
            aOpMode.telemetryAddData("Motor BL", "Values", "" + backLeftMotor.getCurrentPosition());
            aOpMode.telemetryAddData("Motor BR", "Values", "" + backRightMotor.getCurrentPosition());
            aOpMode.telemetryUpdate();

        }
        stopMotors(aOpMode);

        Thread.sleep(100);
    }

    /**
     * Runs motors forwards and backwards.
     *
     * @param Power each motor will run at the same float value
     * @return void
     */
    public void runMotorsFB(vv_OpMode aOpMode, float Power)
            throws InterruptedException {

        runMotors(aOpMode, Power, Power, Power, Power);
    }

    /**
     * Runs motors sideways (right and left).
     *
     * @param Power each motor will run at the same float value
     * @return void
     */
    public void runMotorsSideways(vv_OpMode aOpMode, float Power)
            throws InterruptedException {

        runMotors(aOpMode, -Power, Power, Power, -Power);
    }

    /**
     * Runs motors. Can be called by a more specific method to move forwards and backwards or sideways.
     *
     * @param aOpMode  object of vv_OpMode class so we can use telemetry
     * @param fl_Power power of front left motor
     * @param fr_Power power of front right motor
     * @param bl_Power power of back left motor
     * @param br_Power power of back right motor
     * @return void
     */
    public void runMotors(vv_OpMode aOpMode, float fl_Power, float fr_Power, float bl_Power, float br_Power)
            throws InterruptedException {

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //sets the the power of all motors
        setPower(aOpMode, vv_Constants.MotorEnum.frontLeftMotor, fl_Power);
        setPower(aOpMode, vv_Constants.MotorEnum.frontRightMotor, fr_Power);
        setPower(aOpMode, vv_Constants.MotorEnum.backLeftMotor, bl_Power);
        setPower(aOpMode, vv_Constants.MotorEnum.backRightMotor, br_Power);
    }


    public void stopMotors(vv_OpMode aOpMode) {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void pushButton(vv_OpMode aOpMode, vv_Constants.ButtonEnum buttonEnum) {

        switch (buttonEnum) {

            case Left:
                buttonServo.setPosition(vv_Constants.BUTTON_SERVO_MAX_POS);
                break;

            case Right:
                buttonServo.setPosition(vv_Constants.BUTTON_SERVO_MIN_POS);
                break;
        }
    }

    public boolean getButtonTouchValue(vv_OpMode aOpMode) throws InterruptedException {
        return buttonSensor.isPressed();
    }

    public ColorSensor getColorSensor(vv_OpMode aOpMode) throws InterruptedException {
        return cs;
    }

    public void waitForTick(vv_OpMode aOpMode, long periodMs) throws InterruptedException {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
