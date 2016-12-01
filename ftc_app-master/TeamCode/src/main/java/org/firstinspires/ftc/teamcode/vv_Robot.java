package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.vv_Constants.ARM_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BUTTON_SERVO_MAX_POS;
import static org.firstinspires.ftc.teamcode.vv_Constants.BUTTON_SERVO_MIN_POS;
import static org.firstinspires.ftc.teamcode.vv_Constants.DEBUG;
import static org.firstinspires.ftc.teamcode.vv_Constants.ENCODED_MOTOR_STALL_CLICKS_TETRIX;
import static org.firstinspires.ftc.teamcode.vv_Constants.ENCODED_MOTOR_STALL_TIME_DELTA;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.INTAKE_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.IntakeStateEnum.Off;
import static org.firstinspires.ftc.teamcode.vv_Constants.MAX_MOTOR_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN;
import static org.firstinspires.ftc.teamcode.vv_Constants.MOTOR_LOWER_POWER_THRESHOLD;
import static org.firstinspires.ftc.teamcode.vv_Constants.MOTOR_RAMP_POWER_LOWER_LIMIT;
import static org.firstinspires.ftc.teamcode.vv_Constants.MOTOR_RAMP_POWER_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.vv_Constants.WORM_DRIVE_DURATION_MAX;
import static org.firstinspires.ftc.teamcode.vv_Constants.WORM_DRIVE_ENCODER_MARGIN;
import static org.firstinspires.ftc.teamcode.vv_Constants.WORM_DRIVE_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.WORM_DRIVE_POWER;


/**
 * Created by thomas on 9/25/2016.
 */

public class vv_Robot {
    HardwareMap hwMap = null;
    private DcMotor motorArray[];


    private Servo buttonServo = null;
    private TouchSensor buttonSensor;
    private TouchSensor armSensor;
    private ColorSensor cs;
    private ModernRoboticsI2cGyro base_gyro_sensor;
    private ElapsedTime period = new ElapsedTime();
    private vv_Constants.IntakeStateEnum IntakeState = Off;


    public void init(vv_OpMode aOpMode, HardwareMap ahwMap) throws InterruptedException{
        // save reference to HW Map
        aOpMode.DBG("in Robot init");
        hwMap = ahwMap;

        // Define and Initialize Motors

        //allocate space for our DcMotor Array

        motorArray = new DcMotor[10];

        motorArray[FRONT_LEFT_MOTOR] = hwMap.dcMotor.get("motor_front_left");
        motorArray[FRONT_RIGHT_MOTOR] = hwMap.dcMotor.get("motor_front_right");
        motorArray[BACK_LEFT_MOTOR] = hwMap.dcMotor.get("motor_back_left");
        motorArray[BACK_RIGHT_MOTOR] = hwMap.dcMotor.get("motor_back_right");
        motorArray[ARM_MOTOR] = hwMap.dcMotor.get("motor_arm");
        motorArray[WORM_DRIVE_MOTOR] = hwMap.dcMotor.get("motor_worm");
        motorArray[INTAKE_MOTOR] = hwMap.dcMotor.get("motor_intake");

        cs = hwMap.colorSensor.get("color_line_sensor");

        //turn the LED on the floor color sensor off at the start.
        //used for compatibility with older SDK code.
        cs.enableLed(false);
        //wait for it to turn off.

        Thread.sleep(300);

        aOpMode.DBG("before gyro calib");

        base_gyro_sensor = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("base_gyro_sensor");
        base_gyro_sensor.calibrate();
        while (base_gyro_sensor.isCalibrating()) {
            //wait for calibration completion
            Thread.sleep(50);
            aOpMode.idle();
        }
        aOpMode.DBG("after gyro calib");
        Thread.sleep(2000);


        armSensor = hwMap.touchSensor.get("touch_arm_sensor");

        //buttonServo = hwMap.servo.get("button_servo");

        //buttonSensor = hwMap.touchSensor.get("touch_button_sensor");

        //buttonServo.setPosition(0.65);

        aOpMode.DBG("before motor dir set");

        motorArray[FRONT_LEFT_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);
        motorArray[FRONT_RIGHT_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);
        motorArray[BACK_LEFT_MOTOR].setDirection(DcMotorSimple.Direction.REVERSE);
        motorArray[BACK_RIGHT_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);

        //reset encoders for all encoded motors!!

        motorArray[FRONT_LEFT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[FRONT_RIGHT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[BACK_LEFT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[BACK_LEFT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[WORM_DRIVE_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (motorArray[FRONT_LEFT_MOTOR].getCurrentPosition() != 0) {
            //wait for the Front Left Motor to settle. as a proxy for all of the motors.
            aOpMode.idle();
        }

        // Set all base motors to zero power
        stopBaseMotors(aOpMode);

        aOpMode.DBG("exiting Robot init");

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

    }

    public void setPower(vv_OpMode aOpMode, int motorName, float power) throws MotorNameNotKnownException, InterruptedException {

        motorArray[motorName].setPower(power);
    }

    public DcMotor.RunMode getMotorMode(vv_OpMode aOpMode, int motorName
    ) throws MotorNameNotKnownException, InterruptedException {

        return motorArray[motorName].getMode();

    }

    public void setMotorMode(vv_OpMode aOpMode, int motorName,
                             DcMotor.RunMode runMode) throws MotorNameNotKnownException, InterruptedException {

        motorArray[motorName].setMode(runMode);
    }

    public boolean isArmAtLimit(vv_OpMode aOpMode) {
        return armSensor.isPressed();
        //TODO: Finish this method up
    }

    public void runRobotToPositionFB(vv_OpMode aOpMode, int position, float Power) throws InterruptedException, MotorNameNotKnownException {
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
    public void runRobotToPositionSideways(vv_OpMode aOpMode, int position, float Power) throws InterruptedException, MotorNameNotKnownException {
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
            throws InterruptedException, MotorNameNotKnownException {

        //save the current run mode
        DcMotor.RunMode oldRunMode = motorArray[FRONT_LEFT_MOTOR].getMode();

        //reset motor encoders
        motorArray[FRONT_LEFT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[FRONT_RIGHT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[BACK_LEFT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[BACK_RIGHT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (motorArray[FRONT_LEFT_MOTOR].getCurrentPosition() != 0) {
            //wait until motors are reset
            Thread.sleep(20);
        }

        //sets all motors to run to a position
        motorArray[FRONT_LEFT_MOTOR].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArray[FRONT_RIGHT_MOTOR].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArray[BACK_LEFT_MOTOR].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArray[BACK_RIGHT_MOTOR].setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //reset encoder for 1 wheel
        motorArray[FRONT_LEFT_MOTOR].setTargetPosition(fl_Position);
        motorArray[FRONT_RIGHT_MOTOR].setTargetPosition(fr_Position);
        motorArray[BACK_LEFT_MOTOR].setTargetPosition(bl_Position);
        motorArray[BACK_RIGHT_MOTOR].setTargetPosition(br_Position);


        //sets the the power of all motors
        //since we are ramping up, start at the lowest power allowed.
        setPower(aOpMode, FRONT_LEFT_MOTOR, MOTOR_LOWER_POWER_THRESHOLD);
        setPower(aOpMode, FRONT_RIGHT_MOTOR, MOTOR_LOWER_POWER_THRESHOLD);
        setPower(aOpMode, BACK_LEFT_MOTOR, MOTOR_LOWER_POWER_THRESHOLD);
        setPower(aOpMode, BACK_RIGHT_MOTOR, MOTOR_LOWER_POWER_THRESHOLD);

        //wait until robot reaches target position


        aOpMode.reset_timer();
        while (baseMotorsAreBusy() && (aOpMode.time_elapsed() < MAX_MOTOR_LOOP_TIME)) {
            //wait until motors havce completed movement or timed out.
            //report motor positions for debugging

            //adjust the motor speeds by adjusting Power proportional to distance that needs to be travelled.

            //
            //Ramped Move block formula:
            //RP=PMax(1-4*(0.5-DT/DD)^2)
            //where RP=Ramped Power, PMax is maximum power available, DT=Distance Travelled, DD=Distance to be travelled
            //fl_position (target for the front left motor in encoder clicks can be taken as the proxy for all motors.
            float rampedPowerRaw = (float) (MOTOR_RAMP_POWER_UPPER_LIMIT * (1 - 4 * (Math.pow((0.5f -
                    Math.abs((motorArray[FRONT_LEFT_MOTOR].getCurrentPosition() * 1.0f) / fl_Position)), 2.0f))));

            //use another variable to check and adjust power limits, so we can display raw power values.
            float rampedPower = rampedPowerRaw;

            //check for upper and lower limits.
            if (rampedPower > MOTOR_RAMP_POWER_UPPER_LIMIT) {
                rampedPower = MOTOR_RAMP_POWER_UPPER_LIMIT;
            }
            if (rampedPower < MOTOR_RAMP_POWER_LOWER_LIMIT) {
                rampedPower = MOTOR_RAMP_POWER_LOWER_LIMIT;
            }

            //apply the new power values.
            //sets the the power of all motors

            setPower(aOpMode, FRONT_LEFT_MOTOR, rampedPower);
            setPower(aOpMode, FRONT_RIGHT_MOTOR, rampedPower);
            setPower(aOpMode, BACK_LEFT_MOTOR, rampedPower);
            setPower(aOpMode, BACK_RIGHT_MOTOR, rampedPower);



            // TODO: UNCOMMENT THIS!!!!
            if (DEBUG) {
                aOpMode.telemetryAddData("Motor FL", "Values", ":" + motorArray[FRONT_LEFT_MOTOR].getCurrentPosition());
                aOpMode.telemetryAddData("Motor FR", "Values", ":" + motorArray[FRONT_RIGHT_MOTOR].getCurrentPosition());
                aOpMode.telemetryAddData("Motor BL", "Values", ":" + motorArray[BACK_LEFT_MOTOR].getCurrentPosition());
                aOpMode.telemetryAddData("Motor BR", "Values", ":" + motorArray[BACK_RIGHT_MOTOR].getCurrentPosition());
                aOpMode.telemetryAddData("Raw Ramped Power", "Values", ":" + rampedPowerRaw);
                aOpMode.telemetryAddData("Ramped Power ", "Values", ":" + rampedPower);
                aOpMode.telemetryAddData("fl_position", "Values", ":" + fl_Position);
                aOpMode.telemetryAddData("DTraveled/DTarget", "Values", ":" +
                        Math.abs(motorArray[FRONT_LEFT_MOTOR].getCurrentPosition() / fl_Position));
                aOpMode.telemetryAddData("Squared Values", "Values", ":" +
                        Math.pow((0.5f -
                                Math.abs((motorArray[FRONT_LEFT_MOTOR].getCurrentPosition() * 1.0f) / fl_Position)), 2.0f));


                aOpMode.telemetryUpdate();
            }
            aOpMode.idle();
        }
        stopBaseMotors(aOpMode);

        //restore old run modes
        motorArray[FRONT_LEFT_MOTOR].setMode(oldRunMode);
        motorArray[FRONT_RIGHT_MOTOR].setMode(oldRunMode);
        motorArray[BACK_LEFT_MOTOR].setMode(oldRunMode);
        motorArray[BACK_RIGHT_MOTOR].setMode(oldRunMode);

        Thread.sleep(100);
    }

    public void runRobotToPositionWithAngle(vv_OpMode aOpMode, float fl_Power, float fr_Power,
                                            float bl_Power, float br_Power, int fl_Position,
                                            int fr_Position, int bl_Position, int br_Position, float angle)
            throws InterruptedException, MotorNameNotKnownException {

        //reset motor encoders
        motorArray[FRONT_LEFT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[FRONT_RIGHT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[BACK_LEFT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[BACK_RIGHT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (motorArray[FRONT_LEFT_MOTOR].getCurrentPosition() != 0) {
            //wait until motors are reset
            Thread.sleep(20);
        }

        //sets all motors to run to a position
        try {
            setMotorMode(aOpMode, FRONT_LEFT_MOTOR, DcMotor.RunMode.RUN_TO_POSITION);
            setMotorMode(aOpMode, FRONT_RIGHT_MOTOR, DcMotor.RunMode.RUN_TO_POSITION);
            setMotorMode(aOpMode, BACK_LEFT_MOTOR, DcMotor.RunMode.RUN_TO_POSITION);
            setMotorMode(aOpMode, BACK_RIGHT_MOTOR, DcMotor.RunMode.RUN_TO_POSITION);
            while (motorArray[FRONT_LEFT_MOTOR].isBusy() || motorArray[FRONT_RIGHT_MOTOR].isBusy() ||
                    motorArray[BACK_LEFT_MOTOR].isBusy() || motorArray[BACK_RIGHT_MOTOR].isBusy()) {
                //wait until the motors have finished these tasks
                aOpMode.idle();
            }
        } catch (MotorNameNotKnownException mNNKE) {
            aOpMode.telemetryAddData("Motor Control Error", "Error", mNNKE.getMessage());
        }
        //reset encoders
        motorArray[FRONT_LEFT_MOTOR].setTargetPosition(fl_Position);
        motorArray[FRONT_RIGHT_MOTOR].setTargetPosition(fr_Position);
        motorArray[BACK_LEFT_MOTOR].setTargetPosition(bl_Position);
        motorArray[BACK_RIGHT_MOTOR].setTargetPosition(br_Position);

        //sets the the power of all motors
        setPower(aOpMode, FRONT_LEFT_MOTOR, fl_Power);
        setPower(aOpMode, FRONT_RIGHT_MOTOR, fr_Power);
        setPower(aOpMode, BACK_LEFT_MOTOR, bl_Power);
        setPower(aOpMode, BACK_RIGHT_MOTOR, br_Power);


        //wait until robot reaches target position
        //testing the wheels on the opposite sides of the robot because each might have a different position for sideways movements

        long startTime = System.currentTimeMillis();
        while (baseMotorsAreBusy() && ((System.currentTimeMillis() - startTime)) < MAX_MOTOR_LOOP_TIME) {
            //wait while motors reach targets or we time out.
            //report motor positions for debugging
            aOpMode.telemetryAddData("Motor FL", "Values", "" + motorArray[FRONT_LEFT_MOTOR].getCurrentPosition());
            aOpMode.telemetryAddData("Motor FR", "Values", "" + motorArray[FRONT_RIGHT_MOTOR].getCurrentPosition());
            aOpMode.telemetryAddData("Motor BL", "Values", "" + motorArray[BACK_LEFT_MOTOR].getCurrentPosition());
            aOpMode.telemetryAddData("Motor BR", "Values", "" + motorArray[BACK_RIGHT_MOTOR].getCurrentPosition());
            aOpMode.telemetryUpdate();
            aOpMode.idle();

        }
        stopBaseMotors(aOpMode);
        //final value display
        aOpMode.telemetryAddData("Motor FL", "Values", "" + motorArray[FRONT_LEFT_MOTOR].getCurrentPosition());
        aOpMode.telemetryAddData("Motor FR", "Values", "" + motorArray[FRONT_RIGHT_MOTOR].getCurrentPosition());
        aOpMode.telemetryAddData("Motor BL", "Values", "" + motorArray[BACK_LEFT_MOTOR].getCurrentPosition());
        aOpMode.telemetryAddData("Motor BR", "Values", "" + motorArray[BACK_RIGHT_MOTOR].getCurrentPosition());
        aOpMode.telemetryUpdate();

        Thread.sleep(100);
    }

    /**
     * Runs motors forwards and backwards.
     *
     * @param Power each motor will run at the same float value
     * @return void
     */
    public void runMotorsFB(vv_OpMode aOpMode, float Power)
            throws InterruptedException, MotorNameNotKnownException {

        runMotors(aOpMode, Power, Power, Power, Power);
    }

    /**
     * Runs motors sideways (right and left).
     *
     * @param Power each motor will run at the same float value
     * @return void
     */
    public void runMotorsSideways(vv_OpMode aOpMode, float Power)
            throws InterruptedException, MotorNameNotKnownException {

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
            throws InterruptedException, MotorNameNotKnownException {

        motorArray[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArray[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArray[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArray[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //sets the the power of all motors
        setPower(aOpMode, FRONT_LEFT_MOTOR, fl_Power);
        setPower(aOpMode, FRONT_RIGHT_MOTOR, fr_Power);
        setPower(aOpMode, BACK_LEFT_MOTOR, bl_Power);
        setPower(aOpMode, BACK_RIGHT_MOTOR, br_Power);
    }

    public void stopBaseMotors(vv_OpMode aOpMode) {
        motorArray[FRONT_LEFT_MOTOR].setPower(0);
        motorArray[FRONT_RIGHT_MOTOR].setPower(0);
        motorArray[BACK_LEFT_MOTOR].setPower(0);
        motorArray[BACK_RIGHT_MOTOR].setPower(0);
    }

    public void pushButton(vv_OpMode aOpMode, vv_Constants.BeaconServoStateEnum buttonEnum) {

        switch (buttonEnum) {

            case Left:
                buttonServo.setPosition(BUTTON_SERVO_MAX_POS);
                break;

            case Right:
                buttonServo.setPosition(BUTTON_SERVO_MIN_POS);
                break;
        }
    }

    public boolean getButtonTouchValue(vv_OpMode aOpMode) throws InterruptedException {
        return buttonSensor.isPressed();
    }

    //turn the color sensor LED on the floor of the robot on
    public void enableFloorColorSensorLed(vv_OpMode aOpMode) throws InterruptedException{
        cs.enableLed(true);
        //wait for it to turn on.
        Thread.sleep(300);
    }

    //turn the color sensor LED on the floor of the robot off
    public void disableFloorColorSensorLed(vv_OpMode aOPMode) throws InterruptedException{
        cs.enableLed(false);
        //wait for it to turn off.
        Thread.sleep(300);
    }

    public int getFloorColorSensorAlpha(vv_OpMode aOpMode){
        return cs.alpha();
    }

    //get the alpha (luminosity being read in reflected light from LED)
    //high luminosity will be found with a white surface.

    public int getBaseGyroSensorHeading(vv_OpMode aOpMode) {
        return base_gyro_sensor.getHeading();
    }

    public int getBaseGyroSensorIntegratedZValue(vv_OpMode aOpMode) {
        return base_gyro_sensor.getIntegratedZValue();
    }

    public void resetBaseGyroZIntegrator(vv_OpMode aOpMode) throws InterruptedException {
        base_gyro_sensor.resetZAxisIntegrator();
        Thread.sleep(1000);
    }

    public void waitForTick(vv_OpMode aOpMode, long periodMs) throws InterruptedException {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public boolean baseMotorsAreBusy() {
        return (motorArray[FRONT_LEFT_MOTOR].isBusy() && motorArray[FRONT_RIGHT_MOTOR].isBusy() &&
                motorArray[BACK_LEFT_MOTOR].isBusy() && motorArray[BACK_RIGHT_MOTOR].isBusy());
    }

    public void testMotor(vv_OpMode aOpMode, int motorName, float power, int duration) throws InterruptedException,
            MotorNameNotKnownException {
        aOpMode.DBG("In test motor in vv_robot");

        aOpMode.DBG("after checkname assertion in vv_robot");

        //save the old run mode
        DcMotor.RunMode oldRunMode = motorArray[motorName].getMode();
        aOpMode.DBG("after getmode in vv_robot");

        //change mode to run without encoders

        motorArray[motorName].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //delay for mode change completion
        Thread.sleep(100);

        aOpMode.DBG("after runmode set in vv_robot");

        //set the power to motor
        motorArray[motorName].setPower(power);

        aOpMode.DBG("after power set in vv_robot");

        //reset the timer
        aOpMode.reset_timer();
        while (aOpMode.time_elapsed() < duration) {
            //wait till duration is complete.
            aOpMode.DBG("In motor loop in vv_robot");
            aOpMode.idle();
        }
        //stop the motor

        motorArray[motorName].setPower(0.0f);

        //restore old motor mode
        motorArray[motorName].setMode(oldRunMode);
        //delay to complete switch to old run mode
        Thread.sleep(100);

    }

    public void testEncodedMotor(vv_OpMode aOpMode, int motorName, float power, int maxDuration, int targetPosition) throws InterruptedException,
            MotorNameNotKnownException, MotorStalledException {

        aOpMode.DBG("in testEncodedMotor");


        aOpMode.DBG("after checkname in testEncodedMotor");

        //translate the enumeration to a dc motor
        //save the old run mode
        DcMotor.RunMode oldRunMode = motorArray[motorName].getMode();

        //change the run mode

        motorArray[motorName].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //delay for mode change comopletion

        Thread.sleep(100);

        //set the motor target

        motorArray[motorName].setTargetPosition(targetPosition);

        //set the power to motor
        motorArray[motorName].setPower(power);

        //reset the timer
        aOpMode.reset_timer();

        //variable for stall tests
        long stallTimeStart = 0;
        long stallTimeDelta = 0;
        long stallPositionStart = 0;
        long stallPositionDelta = 0;
        float stallVelocity = 0;
        float stallVelocityThreshold = (ENCODED_MOTOR_STALL_CLICKS_TETRIX * 1.0f /
                ENCODED_MOTOR_STALL_TIME_DELTA);

        aOpMode.DBG("before move loop in testEncodedMotor");

        while ((motorArray[motorName].isBusy()) &&
                ((Math.abs(motorArray[motorName].getCurrentPosition()) - Math.abs(targetPosition)) > MECCANUM_WHEEL_ENCODER_MARGIN) &&
                (aOpMode.time_elapsed() < maxDuration)) {
            stallPositionStart = motorArray[motorName].getCurrentPosition();
            stallTimeStart = aOpMode.time_elapsed();
            //wait till the run is complete or the time runs out.
            Thread.sleep(50);
            aOpMode.idle();
            //stall code
            stallPositionDelta = Math.abs(motorArray[motorName].getCurrentPosition()) - Math.abs(stallPositionStart);
            stallTimeDelta = aOpMode.time_elapsed() - stallTimeStart;
            stallVelocity = ((stallPositionDelta * 1.0f) / stallTimeDelta);

            //TODO: Stall code must be tested!!
            if (stallTimeDelta > 0) {
                if (stallVelocity < stallVelocityThreshold) {
                    //motor stalling ?
                    //stop motor first
                    aOpMode.DBG("in stall code throw testEncodedMotor");
                    motorArray[motorName].setPower(0.0f);
                    //throw exception indicating the problem.
                    throw new MotorStalledException("MotorName" + motorName, stallVelocity, stallVelocityThreshold);
                }
            }


        }
        //stop the motor
        motorArray[motorName].setPower(0.0f);

        //restore old motor mode
        motorArray[motorName].setMode(oldRunMode);
        //delay to complete switch to old run mode
        Thread.sleep(100);

    }

    public int getLauncherPowerPosition(vv_OpMode aOpMode) {
        return motorArray[WORM_DRIVE_MOTOR].getCurrentPosition();
    }

    public void setLauncherPowerPosition(vv_OpMode aOpMode, int launcherPowerPosition)
            throws InterruptedException, MotorStalledException {
        //set the mode to be RUN_TO_POSITION
        //we dont have to save previous state because we will never run the WORM DRIVE motor
        //in any other mode.

        motorArray[WORM_DRIVE_MOTOR].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //allow for a bit of time for set to complete.
        Thread.sleep(50);

        //Now set the target
        motorArray[WORM_DRIVE_MOTOR].setTargetPosition(launcherPowerPosition);

        //now set the power
        motorArray[WORM_DRIVE_MOTOR].setPower(WORM_DRIVE_POWER);

        float stallVelocityThreshold = (ENCODED_MOTOR_STALL_CLICKS_TETRIX * 1.0f /
                ENCODED_MOTOR_STALL_TIME_DELTA);

        //wait till completion of max duration.
        aOpMode.reset_timer();
        while (motorArray[WORM_DRIVE_MOTOR].isBusy() &&
                (Math.abs((Math.abs(motorArray[WORM_DRIVE_MOTOR].getCurrentPosition()) -
                        Math.abs(launcherPowerPosition))) > WORM_DRIVE_ENCODER_MARGIN) &&
                aOpMode.time_elapsed() < WORM_DRIVE_DURATION_MAX) {
            //wait until the position is reached or times out.

            aOpMode.idle();
            int stallPositionStart = motorArray[WORM_DRIVE_MOTOR].getCurrentPosition();
            long stallTimeStart = aOpMode.time_elapsed();
            //wait till the run is complete or the time runs out.
            Thread.sleep(50);
            aOpMode.idle();
            //stall code
            int stallPositionDelta = Math.abs(motorArray[WORM_DRIVE_MOTOR].getCurrentPosition()) - Math.abs(stallPositionStart);
            long stallTimeDelta = aOpMode.time_elapsed() - stallTimeStart;
            float stallVelocity = ((stallPositionDelta * 1.0f) / stallTimeDelta);

            //TODO: Stall code must be tested!!
            if (stallTimeDelta > 0) {
                if (stallVelocity < stallVelocityThreshold) {
                    //motor stalling ?
                    //stop motor first
                    aOpMode.DBG("in stall code throw testEncodedMotor");
                    motorArray[WORM_DRIVE_MOTOR].setPower(0.0f);
                    //throw exception indicating the problem.
                    throw new MotorStalledException("MotorName" + WORM_DRIVE_MOTOR, stallVelocity, stallVelocityThreshold);
                }
            }


        }
        //stop the motor
        motorArray[WORM_DRIVE_MOTOR].setPower(0.0f);

    }



    public vv_Constants.IntakeStateEnum getIntakeState() {
        return IntakeState;
    }

    public void setIntakeState(vv_Constants.IntakeStateEnum IntakeStateValue) {
        IntakeState = IntakeStateValue;
    }

    class MotorNameNotKnownException extends Exception {

        MotorNameNotKnownException(String message) {
            super(message);
        }
    }

    class MotorStalledException extends Exception {

        MotorStalledException(String message, float stallVelocity, float stallVelocityThreshold) {
            super(message + stallVelocity + stallVelocityThreshold);
        }
    }


}