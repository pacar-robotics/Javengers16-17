package org.firstinspires.ftc.teamcode;


import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

import static org.firstinspires.ftc.teamcode.vv_Constants.ANALOG_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.vv_Constants.ARM_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BALL_FLAG_SERVO_LOWERED;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_BLUE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_RED_THRESHOLD;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_LEFT_REST;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_RIGHT_REST;
import static org.firstinspires.ftc.teamcode.vv_Constants.DEBUG;
import static org.firstinspires.ftc.teamcode.vv_Constants.ENCODED_MOTOR_STALL_CLICKS_TETRIX;
import static org.firstinspires.ftc.teamcode.vv_Constants.ENCODED_MOTOR_STALL_TIME_DELTA;
import static org.firstinspires.ftc.teamcode.vv_Constants.FLAG_SERVO;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_GATE_SERVO;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.INTAKE_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.IntakeStateEnum.Off;
import static org.firstinspires.ftc.teamcode.vv_Constants.LAUNCH_FRONT_GATE_SERVO_CLOSED;
import static org.firstinspires.ftc.teamcode.vv_Constants.LAUNCH_FRONT_GATE_SERVO_OPEN;
import static org.firstinspires.ftc.teamcode.vv_Constants.LAUNCH_REAR_GATE_SERVO_CLOSED;
import static org.firstinspires.ftc.teamcode.vv_Constants.LAUNCH_REAR_GATE_SERVO_OPEN;
import static org.firstinspires.ftc.teamcode.vv_Constants.LEFT_BEACON_BUTTON_SERVO;
import static org.firstinspires.ftc.teamcode.vv_Constants.LEFT_MOTOR_TRIM_FACTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.MAX_MOTOR_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.vv_Constants.MAX_ROBOT_TURN_MOTOR_VELOCITY;
import static org.firstinspires.ftc.teamcode.vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN;
import static org.firstinspires.ftc.teamcode.vv_Constants.MECCANUM_WHEEL_FRONT_TRACK_DISTANCE;
import static org.firstinspires.ftc.teamcode.vv_Constants.MECCANUM_WHEEL_SIDE_TRACK_DISTANCE;
import static org.firstinspires.ftc.teamcode.vv_Constants.MIN_ROBOT_TURN_MOTOR_VELOCITY;
import static org.firstinspires.ftc.teamcode.vv_Constants.MOTOR_LOWER_POWER_THRESHOLD;
import static org.firstinspires.ftc.teamcode.vv_Constants.MOTOR_RAMP_FB_POWER_LOWER_LIMIT;
import static org.firstinspires.ftc.teamcode.vv_Constants.MOTOR_RAMP_FB_POWER_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.vv_Constants.MOTOR_RAMP_SIDEWAYS_POWER_LOWER_LIMIT;
import static org.firstinspires.ftc.teamcode.vv_Constants.MOTOR_RAMP_SIDEWAYS_POWER_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.vv_Constants.REAR_GATE_SERVO;
import static org.firstinspires.ftc.teamcode.vv_Constants.RIGHT_BEACON_BUTTON_SERVO;
import static org.firstinspires.ftc.teamcode.vv_Constants.RIGHT_MOTOR_TRIM_FACTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.WORM_DRIVE_DURATION_MAX;
import static org.firstinspires.ftc.teamcode.vv_Constants.WORM_DRIVE_ENCODER_MARGIN;
import static org.firstinspires.ftc.teamcode.vv_Constants.WORM_DRIVE_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.WORM_DRIVE_POWER;



/**
 * Created by thomas on 9/25/2016.
 */

public class vv_Robot {
    private final int NAVX_DIM_I2C_PORT = 2;
    //NavX mxp gyro senor related items
    //NavX mxp related values
    //**
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    private final double TARGET_ANGLE_DEGREES = 90.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    private final int DEVICE_TIMEOUT_MS = 500;
    protected navXPIDController yawPIDController;
    HardwareMap hwMap = null;
    private DcMotor motorArray[];
    private Servo servoArray[];


    private TouchSensor beaconTouchSensor;
    private TouchSensor wormDriveTouchSensor;
    private ColorSensor beaconLeftColorSensor;
    private ColorSensor beaconRightColorSensor;
    private TouchSensor armSensor;
    private LightSensor floorLightSensor;
    private ModernRoboticsI2cGyro baseGyroSensor;
    private OpticalDistanceSensor baseEopdSensor;
    //**
    //**
    private AHRS baseMxpGyroSensor; //NavX MXP gyro


    //**
    //private UltrasonicSensor floorUltrasonicSensor; //replaced by MR range sensor.

    private ModernRoboticsI2cRangeSensor rangeSensor;

    private ElapsedTime period = new ElapsedTime();
    private vv_Constants.IntakeStateEnum IntakeState = Off;


    public void init(vv_OpMode aOpMode, HardwareMap ahwMap) throws InterruptedException {
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

        servoArray = new Servo[6];

        servoArray[LEFT_BEACON_BUTTON_SERVO] = hwMap.servo.get("servo_beacon_left");
        servoArray[RIGHT_BEACON_BUTTON_SERVO] = hwMap.servo.get("servo_beacon_right");
        servoArray[FRONT_GATE_SERVO] = hwMap.servo.get("servo_launcher_front_gate");
        servoArray[REAR_GATE_SERVO] = hwMap.servo.get("servo_launcher_rear_gate");
        servoArray[FLAG_SERVO] = hwMap.servo.get("servo_ball_flag");

        //map Sensors
        floorLightSensor = hwMap.lightSensor.get("floor_light_sensor");
        beaconLeftColorSensor = hwMap.colorSensor.get("beacon_left_color_sensor");
        beaconRightColorSensor = hwMap.colorSensor.get("beacon_right_color_sensor");
        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor");
        wormDriveTouchSensor = hwMap.touchSensor.get("touch_worm_sensor");
        beaconTouchSensor = hwMap.touchSensor.get("touch_beacon_sensor");
        baseEopdSensor = hwMap.opticalDistanceSensor.get("base_eopd_sensor");
        baseGyroSensor = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("base_gyro_sensor");
        baseMxpGyroSensor = AHRS.getInstance(hwMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData);
        armSensor = hwMap.touchSensor.get("touch_arm_sensor");


        //change the internal I2C numbers for the color sensors, since we want to use 2.
        //default color sensor I2C addresses are all hardwared to (3c ?)
        beaconLeftColorSensor.setI2cAddress(I2cAddr.create7bit(0x26));
        beaconRightColorSensor.setI2cAddress(I2cAddr.create7bit(0x2e));


        //turn the LED on the floor color sensor off at the start.
        //used for compatibility with older SDK code.
        floorLightSensor.enableLed(false);
        beaconLeftColorSensor.enableLed(false);
        beaconRightColorSensor.enableLed(false);
        //wait for it to turn off.
        Thread.sleep(300);

        //Gyro Sensors
        aOpMode.DBG("before gyro calib");
        baseGyroSensor.calibrate();
        Thread.sleep(100);

        while (baseGyroSensor.isCalibrating()) {
            //wait for calibration completion
            Thread.sleep(50);
            aOpMode.idle();
        }
        while (baseMxpGyroSensor.isCalibrating()) {
            aOpMode.idle();
            Thread.sleep(50);
            aOpMode.telemetryAddData("1 navX-Device", "Status:",
                    baseMxpGyroSensor.isCalibrating() ?
                            "CALIBRATING" : "Calibration Complete");
            aOpMode.telemetryUpdate();
        }
        aOpMode.DBG("after gyro calib");


        //zero out the yaw value, so this will be the frame of reference for future calls.
        //do not call this for duration of run after this.
        baseMxpGyroSensor.zeroYaw();

        //initialize servos
        servoArray[LEFT_BEACON_BUTTON_SERVO].setPosition(BEACON_SERVO_LEFT_REST);
        servoArray[RIGHT_BEACON_BUTTON_SERVO].setPosition(BEACON_SERVO_RIGHT_REST);
        servoArray[FRONT_GATE_SERVO].setPosition(LAUNCH_FRONT_GATE_SERVO_CLOSED);
        servoArray[REAR_GATE_SERVO].setPosition(LAUNCH_REAR_GATE_SERVO_CLOSED);
        servoArray[FLAG_SERVO].setPosition(BALL_FLAG_SERVO_LOWERED);

        //wait for these servos to reach desired state
        Thread.sleep(100);


        //set motor direction
        aOpMode.DBG("before motor dir set");

        motorArray[FRONT_LEFT_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);
        motorArray[FRONT_RIGHT_MOTOR].setDirection(DcMotorSimple.Direction.REVERSE);
        motorArray[BACK_LEFT_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);
        motorArray[BACK_RIGHT_MOTOR].setDirection(DcMotorSimple.Direction.REVERSE);
        motorArray[WORM_DRIVE_MOTOR].setDirection(DcMotorSimple.Direction.REVERSE);


        //reset encoders for motors always used in encoded mode
        motorArray[WORM_DRIVE_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorArray[CAP_BALL_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        while (motorArray[WORM_DRIVE_MOTOR].getCurrentPosition() != 0) {
            //wait for the Front Left Motor to settle. as a proxy for all of the motors.
            aOpMode.idle();
        }

        //set the run mode to run_to_position for the worm drive
        //since we will not be using it in any other mode.

        motorArray[WORM_DRIVE_MOTOR].setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Set all base motors to zero power
        stopBaseMotors(aOpMode);

        aOpMode.DBG("exiting Robot init");

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        //final initialization countdown to ensure gyro stability if not Field Oriented

        for (int i = 3; i > 0; i--) {
            aOpMode.telemetryAddData("Completing ", " Initialization Countdown->", "[[ " + i + " ]]");
            aOpMode.telemetryUpdate();
            Thread.sleep(1000);
        }
    }

    public void setPower(vv_OpMode aOpMode, int motorName, float power)
            throws InterruptedException {

        motorArray[motorName].setPower(power);
    }

    public DcMotor.RunMode getMotorMode(vv_OpMode aOpMode, int motorName) throws InterruptedException {

        return motorArray[motorName].getMode();

    }

    public void setMotorMode(vv_OpMode aOpMode, int motorName, DcMotor.RunMode runMode)
            throws InterruptedException {

        motorArray[motorName].setMode(runMode);
    }

    public boolean isArmAtLimit(vv_OpMode aOpMode) {
        return armSensor.isPressed();
    }

    public void runRobotCentimeters (vv_OpMode aOpMode, int position,
                                     float Power, boolean isRampedPower, vv_Constants.DirectionEnum direction)
            throws InterruptedException {
        switch (direction) {
            case Forward:
                runRobotToPosition(aOpMode, Power, Power, Power, Power,
                        position, position, position, position, isRampedPower);
                break;
            case Backward:
                runRobotToPosition(aOpMode, Power, Power, Power, Power,
                        position, position, position, position, isRampedPower);
                break;
            case SidewaysLeft:
                runRobotToPosition(aOpMode, Power, Power, Power, Power, -position, position, position, -position, isRampedPower);
                break;
            case SidewaysRight:
                runRobotToPosition(aOpMode, Power, Power, Power, Power, -position, position, position, -position, isRampedPower);
                break;
        }
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
                                   int fr_Position, int bl_Position, int br_Position,
                                   boolean isRampedPower)
            throws InterruptedException {


        double startingPower;
        double rampedPower;
        //reset motor encoders
        motorArray[FRONT_LEFT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[FRONT_RIGHT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[BACK_LEFT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[BACK_RIGHT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        aOpMode.reset_timer();
        while ((motorArray[FRONT_LEFT_MOTOR].getCurrentPosition() != 0) &&
                (motorArray[FRONT_RIGHT_MOTOR].getCurrentPosition() != 0) &&
                (motorArray[BACK_LEFT_MOTOR].getCurrentPosition() != 0) &&
                (motorArray[BACK_RIGHT_MOTOR].getCurrentPosition() != 0) &&
                (aOpMode.time_elapsed() < MAX_MOTOR_LOOP_TIME)
                ) {
            //wait until motors are reset
            aOpMode.idle();
        }

        //sets all motors to run to a position
        motorArray[FRONT_LEFT_MOTOR].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArray[FRONT_RIGHT_MOTOR].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArray[BACK_LEFT_MOTOR].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArray[BACK_RIGHT_MOTOR].setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //set targets
        motorArray[FRONT_LEFT_MOTOR].setTargetPosition(fl_Position);
        motorArray[FRONT_RIGHT_MOTOR].setTargetPosition(fr_Position);
        motorArray[BACK_LEFT_MOTOR].setTargetPosition(bl_Position);
        motorArray[BACK_RIGHT_MOTOR].setTargetPosition(br_Position);


        if (isRampedPower) {
            //sets the the power of all motors
            //since we are ramping up, start at the lowest power allowed.
            setPower(aOpMode, FRONT_LEFT_MOTOR, MOTOR_LOWER_POWER_THRESHOLD);
            setPower(aOpMode, FRONT_RIGHT_MOTOR, MOTOR_LOWER_POWER_THRESHOLD);
            setPower(aOpMode, BACK_LEFT_MOTOR, MOTOR_LOWER_POWER_THRESHOLD);
            setPower(aOpMode, BACK_RIGHT_MOTOR, MOTOR_LOWER_POWER_THRESHOLD);

        } else {
            setPower(aOpMode, FRONT_LEFT_MOTOR, fl_Power * LEFT_MOTOR_TRIM_FACTOR);
            setPower(aOpMode, FRONT_RIGHT_MOTOR, fr_Power * RIGHT_MOTOR_TRIM_FACTOR);
            setPower(aOpMode, BACK_LEFT_MOTOR, bl_Power * LEFT_MOTOR_TRIM_FACTOR);
            setPower(aOpMode, BACK_RIGHT_MOTOR, br_Power * RIGHT_MOTOR_TRIM_FACTOR);
        }


        aOpMode.reset_timer();
        while (baseMotorsAreBusy() && (aOpMode.time_elapsed() < MAX_MOTOR_LOOP_TIME) &&
                (Math.abs(fl_Position - motorArray[FRONT_LEFT_MOTOR].getCurrentPosition())
                        >= MECCANUM_WHEEL_ENCODER_MARGIN)) {
            //wait until motors havce completed movement or timed out.
            //report motor positions for debugging

            //adjust the motor speeds by adjusting Power proportional to distance that needs to be travelled.

            //
            //Ramped Move block formula:
            //RP=PMax(1-4*(0.5-DT/DD)^2)
            //where RP=Ramped Power, PMax is maximum power available, DT=Distance Travelled, DD=Distance to be travelled
            //fl_position (target for the front left motor in encoder clicks can be taken as the proxy for all motors.

            //using fl_power as proxy for all wheel power, the sign is not relevant in this runmode.

            float rampedPowerRaw = (float) (fl_Power * (1 - 4 * (Math.pow((0.5f -
                    Math.abs((motorArray[FRONT_LEFT_MOTOR].getCurrentPosition() * 1.0f) / fl_Position)), 2.0f))));

            //use another variable to check and adjust power limits, so we can display raw power values.
            if (isRampedPower) {
                rampedPower = rampedPowerRaw;
            } else {
                rampedPower = fl_Power; //as proxy for all power.
            }

            if (Math.signum(fl_Position) != Math.signum(fr_Position)) {
                //we are moving sideways, since the front left and right wheels are rotating in opposite directions
                //we should check against sideways limits.
                //check for upper and lower limits.
                if (rampedPower > MOTOR_RAMP_SIDEWAYS_POWER_UPPER_LIMIT) {
                    rampedPower = MOTOR_RAMP_SIDEWAYS_POWER_UPPER_LIMIT;
                }
                if (rampedPower < MOTOR_RAMP_SIDEWAYS_POWER_LOWER_LIMIT) {
                    rampedPower = MOTOR_RAMP_SIDEWAYS_POWER_LOWER_LIMIT;
                }
            } else {
                //else we are moving forward
                //check for upper and lower limits.
                if (rampedPower > MOTOR_RAMP_FB_POWER_UPPER_LIMIT) {
                    rampedPower = MOTOR_RAMP_FB_POWER_UPPER_LIMIT;
                }
                if (rampedPower < MOTOR_RAMP_FB_POWER_LOWER_LIMIT) {
                    rampedPower = MOTOR_RAMP_FB_POWER_LOWER_LIMIT;
                }
            }

            //apply the new power values.
            //sets the the power of all motors

            //in this runmode, the power does not control direction but the sign of the target position does.

            motorArray[FRONT_LEFT_MOTOR].setPower(rampedPower * LEFT_MOTOR_TRIM_FACTOR);
            motorArray[FRONT_RIGHT_MOTOR].setPower(rampedPower * RIGHT_MOTOR_TRIM_FACTOR);
            motorArray[BACK_LEFT_MOTOR].setPower(rampedPower * LEFT_MOTOR_TRIM_FACTOR);
            motorArray[BACK_RIGHT_MOTOR].setPower(rampedPower * RIGHT_MOTOR_TRIM_FACTOR);


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

    }

    public void runRobotToPositionWithAngle(vv_OpMode aOpMode, float fl_Power, float fr_Power,
                                            float bl_Power, float br_Power, int fl_Position,
                                            int fr_Position, int bl_Position, int br_Position, float angle)
            throws InterruptedException {

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

        setMotorMode(aOpMode, FRONT_LEFT_MOTOR, DcMotor.RunMode.RUN_TO_POSITION);
        setMotorMode(aOpMode, FRONT_RIGHT_MOTOR, DcMotor.RunMode.RUN_TO_POSITION);
        setMotorMode(aOpMode, BACK_LEFT_MOTOR, DcMotor.RunMode.RUN_TO_POSITION);
        setMotorMode(aOpMode, BACK_RIGHT_MOTOR, DcMotor.RunMode.RUN_TO_POSITION);
        while (motorArray[FRONT_LEFT_MOTOR].isBusy() || motorArray[FRONT_RIGHT_MOTOR].isBusy() ||
                motorArray[BACK_LEFT_MOTOR].isBusy() || motorArray[BACK_RIGHT_MOTOR].isBusy()) {
            //wait until the motors have finished these tasks
            aOpMode.idle();
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
        while (baseMotorsAreBusy() && ((System.currentTimeMillis() - startTime) < MAX_MOTOR_LOOP_TIME) &&
                (Math.abs(motorArray[FRONT_LEFT_MOTOR].getCurrentPosition() - fl_Position) > MECCANUM_WHEEL_ENCODER_MARGIN)) {
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


    public void runMotorsUsingPower (vv_OpMode aOpMode, float Power, vv_Constants.DirectionEnum direction)
        throws InterruptedException {
        switch (direction){
            case Forward:
                runMotors(aOpMode, Power, Power, Power, Power);
                break;
            case Backward:
                runMotors(aOpMode, Power, Power, Power, Power);
                break;
            case SidewaysLeft:
                runMotors(aOpMode, -Power, Power, Power, -Power);
                break;
            case SidewaysRight:
                runMotors(aOpMode, Power, -Power, -Power, Power);
                break;
        }
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

        motorArray[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[3].setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //sets the the power of all motors
        setPower(aOpMode, FRONT_LEFT_MOTOR, fl_Power);
        setPower(aOpMode, FRONT_RIGHT_MOTOR, fr_Power);
        setPower(aOpMode, BACK_LEFT_MOTOR, bl_Power);
        setPower(aOpMode, BACK_RIGHT_MOTOR, br_Power);
    }

    public void runMotorsUsingEncoders(vv_OpMode aOpMode, float fl_Power, float fr_Power, float bl_Power, float br_Power)
            throws InterruptedException {

        motorArray[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[3].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sets the the power of all motors
        setPower(aOpMode, FRONT_LEFT_MOTOR, limit_power(aOpMode, fl_Power));
        setPower(aOpMode, FRONT_RIGHT_MOTOR, limit_power(aOpMode, fr_Power));
        setPower(aOpMode, BACK_LEFT_MOTOR, limit_power(aOpMode, bl_Power));
        setPower(aOpMode, BACK_RIGHT_MOTOR, limit_power(aOpMode, br_Power));
    }

    public void stopBaseMotors(vv_OpMode aOpMode) throws InterruptedException {

        motorArray[FRONT_LEFT_MOTOR].setPower(0);
        motorArray[FRONT_RIGHT_MOTOR].setPower(0);
        motorArray[BACK_LEFT_MOTOR].setPower(0);
        motorArray[BACK_RIGHT_MOTOR].setPower(0);
        while (motorArray[BACK_RIGHT_MOTOR].getPower() != 0) {
            aOpMode.idle();
        }
    }


    //turn the color sensor LED on the floor of the robot on
    public void enableFloorLightSensorLed(vv_OpMode aOpMode) throws InterruptedException {
        floorLightSensor.enableLed(true);
        //wait for it to turn on.
        Thread.sleep(300);
    }

    //turn the color sensor LED on the floor of the robot off
    public void disableFloorLightSensorLed(vv_OpMode aOPMode) throws InterruptedException {
        floorLightSensor.enableLed(false);
        //wait for it to turn off.
        Thread.sleep(300);
    }

    //turn the color sensor LED on the floor of the robot on
    public void enableBeaconLeftColorSensorLed(vv_OpMode aOpMode) throws InterruptedException {
        beaconLeftColorSensor.enableLed(true);
        //wait for it to turn on.
        Thread.sleep(300);
    }

    //turn the color sensor LED on the floor of the robot off
    public void disableBeaconLeftColorSensorLed(vv_OpMode aOPMode) throws InterruptedException {
        beaconLeftColorSensor.enableLed(false);
        //wait for it to turn off.
        Thread.sleep(300);
    }

    //turn the color sensor LED on the floor of the robot on
    public void enableBeaconRightColorSensorLed(vv_OpMode aOpMode) throws InterruptedException {
        beaconLeftColorSensor.enableLed(true);
        //wait for it to turn on.
        Thread.sleep(300);
    }

    //turn the color sensor LED on the floor of the robot off
    public void disableBeaconRightColorSensorLed(vv_OpMode aOPMode) throws InterruptedException {
        beaconLeftColorSensor.enableLed(false);
        //wait for it to turn off.
        Thread.sleep(300);
    }


    public vv_Constants.BeaconColorEnum getBeaconLeftColor(vv_OpMode aOpMode) throws InterruptedException {
        Thread.sleep(30);
        if ((beaconLeftColorSensor.red() > BEACON_RED_THRESHOLD) &&
                (beaconLeftColorSensor.red() > beaconLeftColorSensor.blue())) {
            return vv_Constants.BeaconColorEnum.RED;
        }
        if ((beaconLeftColorSensor.blue() > BEACON_BLUE_THRESHOLD) &&
                (beaconLeftColorSensor.blue() > beaconLeftColorSensor.red())) {
            return vv_Constants.BeaconColorEnum.BLUE;
        }

        return vv_Constants.BeaconColorEnum.UNKNOWN;
    }

    public vv_Constants.BeaconColorEnum getBeaconRightColor(vv_OpMode aOpMode) throws InterruptedException {
        Thread.sleep(30);
        if ((beaconRightColorSensor.red() > BEACON_RED_THRESHOLD) &&
                (beaconRightColorSensor.red() > beaconRightColorSensor.blue())) {
            return vv_Constants.BeaconColorEnum.RED;
        }
        if ((beaconRightColorSensor.blue() > BEACON_BLUE_THRESHOLD) &&
                (beaconRightColorSensor.blue() > beaconRightColorSensor.red())) {
            return vv_Constants.BeaconColorEnum.BLUE;
        }

        return vv_Constants.BeaconColorEnum.UNKNOWN;
    }


    //get the alpha (luminosity being read in reflected light from LED)
    //high luminosity will be found with a white surface.

    public int getBaseGyroSensorHeading(vv_OpMode aOpMode) {
        return baseGyroSensor.getHeading();
    }

    public float getMxpGyroSensorHeading(vv_OpMode aOpMode) {
        return baseMxpGyroSensor.getYaw();
    }

    public float getMxpFusedGyroSensorHeading(vv_OpMode aOpMode) {
        return baseMxpGyroSensor.getFusedHeading();
    }


    public int getBaseGyroSensorIntegratedZValue(vv_OpMode aOpMode) {
        return baseGyroSensor.getIntegratedZValue();
    }

    public void resetBaseGyroZIntegrator(vv_OpMode aOpMode) throws InterruptedException {
        baseGyroSensor.resetZAxisIntegrator();
        Thread.sleep(1000);
    }


    public boolean baseMotorsAreBusy() {
        return (motorArray[FRONT_LEFT_MOTOR].isBusy() || motorArray[FRONT_RIGHT_MOTOR].isBusy() ||
                motorArray[BACK_LEFT_MOTOR].isBusy() || motorArray[BACK_RIGHT_MOTOR].isBusy());
    }

    public void testMotor(vv_OpMode aOpMode, int motorName, float power, long duration) throws InterruptedException {
        aOpMode.DBG("In test motor in vv_robot");

        aOpMode.DBG("after checkname assertion in vv_robot");

        //save the old run mode
        DcMotor.RunMode oldRunMode = motorArray[motorName].getMode();
        aOpMode.DBG("after getmode in vv_robot");

        //change mode to run without encoders

        motorArray[motorName].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //delay for mode change completion
        Thread.sleep(1000);

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
        Thread.sleep(1000);

    }

    public void testEncodedMotor(vv_OpMode aOpMode, int motorName, float power, int maxDuration, int targetPosition) throws InterruptedException,
            MotorStalledException {

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
                ((Math.abs(motorArray[motorName].getCurrentPosition() - targetPosition)) > MECCANUM_WHEEL_ENCODER_MARGIN) &&
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
                    String stallMessage = "MotorName" + motorName + "StallV:[" +
                            stallVelocity + "]" + "Stall Velocity Threshold:{" +
                            stallVelocityThreshold + "]";
                    throw new MotorStalledException(stallMessage);
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

        //reset clock;
        aOpMode.reset_timer();

        //initialize the variables we need.
        int newStallPosition = 0;
        int oldStallPosition = 0;
        long newStallTime = 0;
        long oldStallTime = 0;
        int stallPositionDelta = 0;
        long stallTimeDelta = 0;
        float stallVelocity = 0;

        while (motorArray[WORM_DRIVE_MOTOR].isBusy() &&
                (aOpMode.time_elapsed() < WORM_DRIVE_DURATION_MAX) &&
                (!wormDriveTouchSensor.isPressed())) {

            //save old stall time and position.
            oldStallPosition = newStallPosition;
            oldStallTime = newStallTime;

            //read the current position only once.
            newStallPosition = motorArray[WORM_DRIVE_MOTOR].getCurrentPosition();
            newStallTime = aOpMode.time_elapsed();

            stallPositionDelta = Math.abs(Math.abs(newStallPosition) - Math.abs(oldStallPosition));
            stallTimeDelta = newStallTime - oldStallTime;

            stallVelocity = ((stallPositionDelta * 1.0f) / stallTimeDelta);

            //TODO: Stall code must be tested!!
            if ((oldStallTime > 0) && stallVelocity < stallVelocityThreshold) {
                //motor stalling ?
                //stop motor first

                motorArray[WORM_DRIVE_MOTOR].setPower(0.0f);
                //throw exception indicating the problem.
                String stallMessage = "stallV:[" + stallVelocity + "]"
                        + "stallT:[" + stallVelocityThreshold + "]"
                        + "stallTDelta:[" + stallTimeDelta + "]"
                        + "stallPDelta:[" + stallPositionDelta + "]";

                if (DEBUG) {
                    aOpMode.DBG("in stall code throw testEncodedMotor");
                    aOpMode.telemetryAddData("Stall Data", "", stallMessage);
                    aOpMode.telemetryUpdate();
                }
                throw new MotorStalledException("MotorName" + ":WormDriveMotor" + stallMessage);
            }


            if ((Math.abs(launcherPowerPosition -
                    newStallPosition)) <= WORM_DRIVE_ENCODER_MARGIN) {
                //stop the motor
                motorArray[WORM_DRIVE_MOTOR].setPower(0.0f);
                //break out of loop, as we have reached target within margin.
                aOpMode.telemetryAddData("Break out of loop, margin", "", "margin");
                aOpMode.telemetryUpdate();
                Thread.sleep(500);

                break;
            }

            //wait for a bit of time to test for stall
            Thread.sleep(ENCODED_MOTOR_STALL_TIME_DELTA);


            aOpMode.idle();
        }
        //stop the motor
        motorArray[WORM_DRIVE_MOTOR].setPower(0.0f);

    }

    public double getFloorLightIntensity(vv_OpMode aOpMode) {
        return floorLightSensor.getLightDetected();

    }

    public void openFrontLauncherGate() throws InterruptedException {
        servoArray[FRONT_GATE_SERVO].setPosition(LAUNCH_FRONT_GATE_SERVO_OPEN);
        Thread.sleep(100);
    }

    public void closeFrontLauncherGate() throws InterruptedException {
        servoArray[FRONT_GATE_SERVO].setPosition(LAUNCH_FRONT_GATE_SERVO_CLOSED);
        Thread.sleep(100);
    }

    public void openRearLauncherGate() throws InterruptedException {
        servoArray[REAR_GATE_SERVO].setPosition(LAUNCH_REAR_GATE_SERVO_OPEN);
        Thread.sleep(100);
    }

    public void closeRearLauncherGate() throws InterruptedException {
        servoArray[REAR_GATE_SERVO].setPosition(LAUNCH_REAR_GATE_SERVO_CLOSED);
        Thread.sleep(100);
    }

    public double getFrontLauncherGateServoPosition(vv_OpMode aOpMode) {
        return servoArray[FRONT_GATE_SERVO].getPosition();
    }

    public void setFrontLauncherGateServoPosition(vv_OpMode aOpMode, double position) {
        servoArray[FRONT_GATE_SERVO].setPosition(position);
    }

    public int getBeaconLeftColorRedValue(vv_OpMode aOpMode) {

        return beaconLeftColorSensor.red();
    }

    public int getBeaconLeftColorGreenValue(vv_OpMode aOpMode) {

        return beaconLeftColorSensor.green();
    }

    public int getBeaconLeftColorBlueValue(vv_OpMode aOpMode) {
        return beaconLeftColorSensor.blue();
    }

    public int getBeaconRightColorRedValue(vv_OpMode aOpMode) {

        return beaconLeftColorSensor.red();
    }

    public int getBeaconRightColorGreenValue(vv_OpMode aOpMode) {

        return beaconLeftColorSensor.green();
    }

    public int getBeaconRightColorBlueValue(vv_OpMode aOpMode) {
        return beaconLeftColorSensor.blue();
    }


    public void setServoPosition (vv_OpMode aOpMode, int servoName, double position) throws InterruptedException {
        switch (servoName) {
            case LEFT_BEACON_BUTTON_SERVO:
                servoArray[LEFT_BEACON_BUTTON_SERVO].setPosition(position);
                break;
            case RIGHT_BEACON_BUTTON_SERVO:
                servoArray[RIGHT_BEACON_BUTTON_SERVO].setPosition(position);
                break;
            case FRONT_GATE_SERVO:
                servoArray[FRONT_GATE_SERVO].setPosition(position);
                break;
            case REAR_GATE_SERVO:
                servoArray[REAR_GATE_SERVO].setPosition(position);
                break;
            case FLAG_SERVO:
                servoArray[FLAG_SERVO].setPosition(position);
                break;
        }
    }

    public double getServoPosition (vv_OpMode aOpMode, int servoName) {
        //WILL RETURN AN ERROR
        double position = 6;

        switch (servoName) {
            case LEFT_BEACON_BUTTON_SERVO:
                position =  servoArray[LEFT_BEACON_BUTTON_SERVO].getPosition();
                break;
            case RIGHT_BEACON_BUTTON_SERVO:
                position =  servoArray[RIGHT_BEACON_BUTTON_SERVO].getPosition();
                break;
            case FRONT_GATE_SERVO:
                position =  servoArray[FRONT_GATE_SERVO].getPosition();
                break;
            case REAR_GATE_SERVO:
                position =  servoArray[REAR_GATE_SERVO].getPosition();
                break;
            case FLAG_SERVO:
                position =  servoArray[FLAG_SERVO].getPosition();
                break;
        }
        return position;
    }

    public vv_Constants.IntakeStateEnum getIntakeState() {
        return IntakeState;
    }

    public void setIntakeState(vv_Constants.IntakeStateEnum IntakeStateValue) {
        IntakeState = IntakeStateValue;
    }


    public boolean isBeaconTouchSensorPressed(vv_OpMode aOpMode) {
        return beaconTouchSensor.isPressed();
    }

    public double getUltrasonicDistance(vv_OpMode aOpMode) {
        return rangeSensor.cmUltrasonic() / 2.54; //in inches
    }

    public double getOpticalDistance(vv_OpMode aOpMode) {
        return rangeSensor.cmOptical() / 2.54; //in inches
    }

    public void universalMoveRobotForDuration(vv_OpMode aOpMode, double xAxisVelocity,
                                              double yAxisVelocity, double rotationalVelocity, long duration)
            throws InterruptedException {
        double fl_velocity = 0;
        double fr_velocity = 0;
        double bl_velocity = 0;
        double br_velocity = 0;
        double trackDistanceAverage = (MECCANUM_WHEEL_FRONT_TRACK_DISTANCE +
                MECCANUM_WHEEL_SIDE_TRACK_DISTANCE) / 2.0f;

        //calculate velocities at each wheel.


        fl_velocity = xAxisVelocity + yAxisVelocity - rotationalVelocity *
                trackDistanceAverage;

        fr_velocity = xAxisVelocity - yAxisVelocity + rotationalVelocity *
                trackDistanceAverage;

        bl_velocity = xAxisVelocity - yAxisVelocity - rotationalVelocity *
                trackDistanceAverage;

        br_velocity = xAxisVelocity + yAxisVelocity + rotationalVelocity *
                trackDistanceAverage;

        //reset all encoders.

        motorArray[FRONT_LEFT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[FRONT_RIGHT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[BACK_LEFT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[BACK_RIGHT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (motorArray[FRONT_LEFT_MOTOR].getCurrentPosition() != 0) {
            //wait for switch to happen.
            aOpMode.idle();
        }
        //switch to RUN_WITH_ENCODERS to normalize for speed.

        motorArray[FRONT_LEFT_MOTOR].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[FRONT_RIGHT_MOTOR].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[BACK_LEFT_MOTOR].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[BACK_RIGHT_MOTOR].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //wait for switch to happen
        Thread.sleep(100);

        //start slow to prevent skid.
        //may replace with ramp to improve performance.

        motorArray[FRONT_LEFT_MOTOR].setPower(Math.abs(fl_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(fl_velocity) * MOTOR_LOWER_POWER_THRESHOLD : fl_velocity);
        motorArray[FRONT_RIGHT_MOTOR].setPower(Math.abs(fr_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(fr_velocity) * MOTOR_LOWER_POWER_THRESHOLD : fr_velocity);
        motorArray[BACK_LEFT_MOTOR].setPower(Math.abs(bl_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(bl_velocity) * MOTOR_LOWER_POWER_THRESHOLD : bl_velocity);
        motorArray[BACK_RIGHT_MOTOR].setPower(Math.abs(br_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(br_velocity) * MOTOR_LOWER_POWER_THRESHOLD : br_velocity);

        //

        aOpMode.reset_timer();
        //stop 100 ms before end
        while (aOpMode.time_elapsed() < duration - 100) {

            //apply specific powers to motors to get desired movement
            //wait till duration is complete.
            motorArray[FRONT_LEFT_MOTOR].setPower(fl_velocity);
            motorArray[FRONT_RIGHT_MOTOR].setPower(fr_velocity);
            motorArray[BACK_LEFT_MOTOR].setPower(bl_velocity);
            motorArray[BACK_RIGHT_MOTOR].setPower(br_velocity);
            aOpMode.idle();
        }

        //end slow
        motorArray[FRONT_LEFT_MOTOR].setPower(Math.abs(fl_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(fl_velocity) * MOTOR_LOWER_POWER_THRESHOLD : fl_velocity);
        motorArray[FRONT_RIGHT_MOTOR].setPower(Math.abs(fr_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(fr_velocity) * MOTOR_LOWER_POWER_THRESHOLD : fr_velocity);
        motorArray[BACK_LEFT_MOTOR].setPower(Math.abs(bl_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(bl_velocity) * MOTOR_LOWER_POWER_THRESHOLD : bl_velocity);
        motorArray[BACK_RIGHT_MOTOR].setPower(Math.abs(br_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(br_velocity) * MOTOR_LOWER_POWER_THRESHOLD : br_velocity);

        aOpMode.reset_timer();
        //stop 100 ms before end
        while (aOpMode.time_elapsed() < 100) {
            aOpMode.idle();
        }


        //stop all motors
        stopBaseMotors(aOpMode);


    }

    public void universalGyroStabilizedMoveRobot(vv_OpMode aOpMode, double xAxisVelocity,
                                                 double yAxisVelocity,
                                                 long duration, vv_OpMode.StopCondition condition)
            throws InterruptedException {


        //PID constants
        final double YAW_PID_P = 0.01;
        final double YAW_PID_I = 0.0;
        final double YAW_PID_D = 0.0;

        //create a PID controller that uses YAW data.
        navXPIDController mxpPidController = new navXPIDController(baseMxpGyroSensor,
                navXPIDController.navXTimestampedDataSource.YAW);







        double fl_velocity = 0;
        double fr_velocity = 0;
        double bl_velocity = 0;
        double br_velocity = 0;
        double trackDistanceAverage = (MECCANUM_WHEEL_FRONT_TRACK_DISTANCE +
                MECCANUM_WHEEL_SIDE_TRACK_DISTANCE) / 2.0f;


        //calculate velocities at each wheel.
        //no rotation components

        fl_velocity = yAxisVelocity + xAxisVelocity;

        fr_velocity = yAxisVelocity - xAxisVelocity;

        bl_velocity = yAxisVelocity - xAxisVelocity;

        br_velocity = yAxisVelocity + xAxisVelocity;

        //reset all encoders.

        motorArray[FRONT_LEFT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[FRONT_RIGHT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[BACK_LEFT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[BACK_RIGHT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (motorArray[FRONT_LEFT_MOTOR].getCurrentPosition() != 0) {
            //wait for switch to happen.
            aOpMode.idle();
        }
        //switch to RUN_WITH_ENCODERS to normalize for speed.

        motorArray[FRONT_LEFT_MOTOR].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[FRONT_RIGHT_MOTOR].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[BACK_LEFT_MOTOR].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[BACK_RIGHT_MOTOR].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //wait for switch to happen
        Thread.sleep(100);


        //we want to go straight, from this point


        //set the target angle to be the current angle.
        //we will try to stay on this path, while trimming to maintain the angle.


        mxpPidController.setSetpoint(baseMxpGyroSensor.getYaw());
        mxpPidController.setOutputRange(-1.0, 1.0);
        mxpPidController.setContinuous(true);
        mxpPidController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, 1.0f);
        mxpPidController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);

        //lets enable the PIDController

        mxpPidController.enable(true);

        navXPIDController.PIDResult mxpPIDResult = new navXPIDController.PIDResult();

        aOpMode.reset_timer();
        while ((aOpMode.time_elapsed() < duration) &&
                (!condition.StopCondition(aOpMode))) {
            if (mxpPidController.waitForNewUpdate(mxpPIDResult, DEVICE_TIMEOUT_MS)) {
                if (mxpPIDResult.isOnTarget()) {

                    motorArray[FRONT_RIGHT_MOTOR].setPower(fr_velocity);
                    motorArray[FRONT_LEFT_MOTOR].setPower(fl_velocity * 0.95);
                    motorArray[BACK_RIGHT_MOTOR].setPower(br_velocity);
                    motorArray[BACK_LEFT_MOTOR].setPower(bl_velocity * 0.95);

                    aOpMode.telemetryAddData("PID:", "Value:", "On track");
                    aOpMode.telemetryUpdate();

                } else {
                    double output = mxpPIDResult.getOutput();
                    motorArray[FRONT_LEFT_MOTOR].setPower(limit_power(aOpMode, (float) (fl_velocity + output)));
                    motorArray[FRONT_RIGHT_MOTOR].setPower(limit_power(aOpMode, (float) (fr_velocity - output)));
                    motorArray[BACK_LEFT_MOTOR].setPower(limit_power(aOpMode, (float) (bl_velocity + output)));
                    motorArray[BACK_RIGHT_MOTOR].setPower(limit_power(aOpMode, (float) (br_velocity - output)));
                    aOpMode.telemetryAddData("PID:", "Value:", "Output degrees:" + output);
                    aOpMode.telemetryUpdate();

                }
            } else {
                    /* A timeout occurred */
                Log.w("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            }

            //condition will return true when it reaches state meant to stop movement

            //apply specific powers to motors to get desired movement
            //wait till duration is complete.


            Thread.sleep(25); //sleep for loop control to Android and update from mxp gyro
            aOpMode.idle();
        }



        //stop all motors
        stopBaseMotors(aOpMode);


    }

    public void universalMoveRobot(vv_OpMode aOpMode, double xAxisVelocity,
                                   double yAxisVelocity, double rotationalVelocity,
                                   long duration, vv_OpMode.StopCondition condition,
                                   boolean isPulsed, long pulseWidthDuration, long pulseRestDuration)
            throws InterruptedException {
        double fl_velocity = 0;
        double fr_velocity = 0;
        double bl_velocity = 0;
        double br_velocity = 0;
        double trackDistanceAverage = (MECCANUM_WHEEL_FRONT_TRACK_DISTANCE +
                MECCANUM_WHEEL_SIDE_TRACK_DISTANCE) / 2.0f;


        //calculate velocities at each wheel.

        fl_velocity = yAxisVelocity + xAxisVelocity - rotationalVelocity *
                trackDistanceAverage;

        fr_velocity = yAxisVelocity - xAxisVelocity + rotationalVelocity *
                trackDistanceAverage;

        bl_velocity = yAxisVelocity - xAxisVelocity - rotationalVelocity *
                trackDistanceAverage;

        br_velocity = yAxisVelocity + xAxisVelocity + rotationalVelocity *
                trackDistanceAverage;

        //reset all encoders.

        motorArray[FRONT_LEFT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[FRONT_RIGHT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[BACK_LEFT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[BACK_RIGHT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (motorArray[FRONT_LEFT_MOTOR].getCurrentPosition() != 0) {
            //wait for switch to happen.
            aOpMode.idle();
        }
        //switch to RUN_WITH_ENCODERS to normalize for speed.

        motorArray[FRONT_LEFT_MOTOR].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[FRONT_RIGHT_MOTOR].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[BACK_LEFT_MOTOR].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[BACK_RIGHT_MOTOR].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //wait for switch to happen
        Thread.sleep(100);

        //start slow to prevent skid.
        //may replace with ramp to improve performance.

        motorArray[FRONT_LEFT_MOTOR].setPower(Math.abs(fl_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(fl_velocity) * MOTOR_LOWER_POWER_THRESHOLD : fl_velocity * LEFT_MOTOR_TRIM_FACTOR);
        motorArray[FRONT_RIGHT_MOTOR].setPower(Math.abs(fr_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(fr_velocity) * MOTOR_LOWER_POWER_THRESHOLD : fr_velocity * RIGHT_MOTOR_TRIM_FACTOR);
        motorArray[BACK_LEFT_MOTOR].setPower(Math.abs(bl_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(bl_velocity) * MOTOR_LOWER_POWER_THRESHOLD : bl_velocity * LEFT_MOTOR_TRIM_FACTOR);
        motorArray[BACK_RIGHT_MOTOR].setPower(Math.abs(br_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(br_velocity) * MOTOR_LOWER_POWER_THRESHOLD : br_velocity * RIGHT_MOTOR_TRIM_FACTOR);

        //

        aOpMode.reset_timer();
        //stop 100 ms before end
        while ((aOpMode.time_elapsed() < (duration - 100)) &&
                (!condition.StopCondition(aOpMode))) {

            //condition will return true when it reaches state meant to stop movement

            //apply specific powers to motors to get desired movement
            //wait till duration is complete.
            motorArray[FRONT_LEFT_MOTOR].setPower(fl_velocity * LEFT_MOTOR_TRIM_FACTOR);
            motorArray[FRONT_RIGHT_MOTOR].setPower(fr_velocity * RIGHT_MOTOR_TRIM_FACTOR);
            motorArray[BACK_LEFT_MOTOR].setPower(bl_velocity * LEFT_MOTOR_TRIM_FACTOR);
            motorArray[BACK_RIGHT_MOTOR].setPower(br_velocity * RIGHT_MOTOR_TRIM_FACTOR);

            if (isPulsed) {
                //run the motors for the pulseWidthDuration
                //by sleeping, we let the motors that are running to continue to run
                Thread.sleep(pulseWidthDuration);

                //stop motors
                stopBaseMotors(aOpMode);
                //pause for pulseRestDuration
                Thread.sleep(pulseRestDuration);
                //this allows the robot to move slowly and gives the sensor time to read
            }
            aOpMode.idle();
        }

        //end slow
        motorArray[FRONT_LEFT_MOTOR].setPower(Math.abs(fl_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(fl_velocity) * MOTOR_LOWER_POWER_THRESHOLD : fl_velocity * LEFT_MOTOR_TRIM_FACTOR);
        motorArray[FRONT_RIGHT_MOTOR].setPower(Math.abs(fr_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(fr_velocity) * MOTOR_LOWER_POWER_THRESHOLD : fr_velocity * RIGHT_MOTOR_TRIM_FACTOR);
        motorArray[BACK_LEFT_MOTOR].setPower(Math.abs(bl_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(bl_velocity) * MOTOR_LOWER_POWER_THRESHOLD : bl_velocity * LEFT_MOTOR_TRIM_FACTOR);
        motorArray[BACK_RIGHT_MOTOR].setPower(Math.abs(br_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(br_velocity) * MOTOR_LOWER_POWER_THRESHOLD : br_velocity * RIGHT_MOTOR_TRIM_FACTOR);

        aOpMode.reset_timer();
        //stop 100 ms before end
        while (aOpMode.time_elapsed() < 100) {
            aOpMode.idle();
        }


        //stop all motors
        stopBaseMotors(aOpMode);


    }

    public void universalMoveRobotForTeleOp(vv_OpMode aOpMode, double xAxisVelocity,
                                            double yAxisVelocity)
            throws InterruptedException {
        double fl_velocity = 0;
        double fr_velocity = 0;
        double bl_velocity = 0;
        double br_velocity = 0;
        double trackDistanceAverage = (MECCANUM_WHEEL_FRONT_TRACK_DISTANCE +
                MECCANUM_WHEEL_SIDE_TRACK_DISTANCE) / 2.0f;


        //calculate velocities at each wheel.

        fl_velocity = yAxisVelocity + xAxisVelocity;

        fr_velocity = yAxisVelocity - xAxisVelocity;

        bl_velocity = yAxisVelocity - xAxisVelocity;

        br_velocity = yAxisVelocity + xAxisVelocity;


        motorArray[FRONT_LEFT_MOTOR].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArray[FRONT_RIGHT_MOTOR].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArray[BACK_LEFT_MOTOR].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArray[BACK_RIGHT_MOTOR].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //wait for switch to happen
        Thread.sleep(50);

        //apply specific powers to motors to get desired movement
        //wait till duration is complete.
        motorArray[FRONT_LEFT_MOTOR].setPower(fl_velocity * LEFT_MOTOR_TRIM_FACTOR);
        motorArray[FRONT_RIGHT_MOTOR].setPower(fr_velocity * RIGHT_MOTOR_TRIM_FACTOR);
        motorArray[BACK_LEFT_MOTOR].setPower(bl_velocity * LEFT_MOTOR_TRIM_FACTOR);
        motorArray[BACK_RIGHT_MOTOR].setPower(br_velocity * RIGHT_MOTOR_TRIM_FACTOR);


    }



    public float limit_power(vv_OpMode aOpMode, float power) {
        //Check and limit the power being applied to motors during turns
        if (power == 0) {
            return 0;
        } else {
            return ((power < MIN_ROBOT_TURN_MOTOR_VELOCITY ? MIN_ROBOT_TURN_MOTOR_VELOCITY :
                    (power > MAX_ROBOT_TURN_MOTOR_VELOCITY ? MAX_ROBOT_TURN_MOTOR_VELOCITY : power)));
        }
    }

    public void turnPidMxpAbsoluteDegrees(vv_OpMode aOpMode, float turndegrees, float toleranceDegrees)
            throws InterruptedException {
         /* Create a PID Controller which uses the Yaw Angle as input. */
        //by default the PIDController is disabled. turn it on.

        // the mxp gyro sensor classes include a built in
        // proportional Integrated Derivative (PID) adjusted control loop function
        //lets set that up for reading the YAW value (rotation around the z axis for the robot).

        yawPIDController = new navXPIDController(baseMxpGyroSensor,
                navXPIDController.navXTimestampedDataSource.YAW);

        //Configure the PID controller for the turn degrees we want
        yawPIDController.setSetpoint(turndegrees);
        yawPIDController.setContinuous(true);

        //limits of motor values (-1.0 to 1.0)
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        //tolerance degrees is defined to prevent oscillation at high accuracy levels.
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, toleranceDegrees);
        //PID initial parameters, usually found by trial and error.

        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);

        yawPIDController.enable(true);


        /* Wait for new Yaw PID output values, then update the motors
           with the new PID value with each new output value.
         */

        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        DecimalFormat df = new DecimalFormat("#.##");

        aOpMode.reset_timer();

        while ((aOpMode.time_elapsed() < MAX_MOTOR_LOOP_TIME) &&
                !Thread.currentThread().isInterrupted()) {
            if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                if (yawPIDResult.isOnTarget()) {
                    //we have reached turn target within tolerance requested.
                    //stop
                    aOpMode.telemetryAddData("On Target", ":Value:",
                            df.format(getMxpGyroSensorHeading(aOpMode)));
                    aOpMode.telemetryUpdate();
                    break;
                } else {
                    //get the new adjustment for direction from the PID Controller
                    float output = (float) yawPIDResult.getOutput();
                    //apply it to the motors, using one of our functions.
                    //if output was positive, the method below would turn the robot clockwise
                    runMotorsUsingEncoders(aOpMode, output, -output, output, -output);
                    aOpMode.telemetryAddData("PIDOutput", ":Value:", df.format(output) + ", " +
                            df.format(-output));
                    aOpMode.telemetryUpdate();
                }
            } else {
                /* A timeout occurred */
                Log.w("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            }
            aOpMode.idle();
            aOpMode.telemetryAddData("Yaw", ":Value:", df.format(getMxpGyroSensorHeading(aOpMode)));
            aOpMode.idle();
        }

    }

    public double getGamePad1RightJoystickPolarMagnitude(vv_OpMode aOpMode) {
        //returns the magnitude of the polar vector for the rotation calculations
        //for field oriented drive
        //inverted y
        if ((Math.abs(aOpMode.gamepad1.right_stick_x) > ANALOG_STICK_THRESHOLD) ||
                (Math.abs(aOpMode.gamepad1.right_stick_y) > ANALOG_STICK_THRESHOLD)) {
            return (Math.sqrt(Math.pow(aOpMode.gamepad1.right_stick_x, 2.0) +
                    Math.pow(-aOpMode.gamepad1.right_stick_y, 2.0)));
        } else {
            return 0;
        }

    }

    public double getGamePad1RightJoystickPolarAngle(vv_OpMode aOpMode) {
        //returns polar angle in degrees of vector for the rotation calculations
        //for field oriented drive.
        //inverted y
        if ((Math.abs(aOpMode.gamepad1.right_stick_x) > ANALOG_STICK_THRESHOLD) ||
                (Math.abs(aOpMode.gamepad1.right_stick_y) > ANALOG_STICK_THRESHOLD)) {
            return (Math.toDegrees(Math.atan2(aOpMode.gamepad1.right_stick_x,
                    -aOpMode.gamepad1.right_stick_y)));
        } else {
            return 0;
        }
    }

    public double getGamePad1LeftJoystickPolarMagnitude(vv_OpMode aOpMode) {
        //returns the magnitude of the polar vector for the rotation calculations
        //for field oriented drive
        //inverted y
        if ((Math.abs(aOpMode.gamepad1.left_stick_x) > ANALOG_STICK_THRESHOLD) ||
                (Math.abs(aOpMode.gamepad1.left_stick_y) > ANALOG_STICK_THRESHOLD)) {
            return (Math.sqrt(Math.pow(aOpMode.gamepad1.left_stick_x, 2.0) +
                    Math.pow(-aOpMode.gamepad1.left_stick_y, 2.0)));
        } else {
            return 0;
        }
    }

    public double getGamePad1LeftJoystickPolarAngle(vv_OpMode aOpMode) {
        //returns polar angle in degrees of vector for the rotation calculations
        //for field oriented drive.
        //inverted y
        if ((Math.abs(aOpMode.gamepad1.left_stick_x) > ANALOG_STICK_THRESHOLD) ||
                (Math.abs(aOpMode.gamepad1.left_stick_y) > ANALOG_STICK_THRESHOLD)) {
            return (Math.toDegrees(Math.atan2(aOpMode.gamepad1.left_stick_x,
                    -aOpMode.gamepad1.left_stick_y)));
        } else {
            return 0;
        }
    }


    public double getEopdRawValue(vv_OpMode aOpMode) {
        return baseEopdSensor.getRawLightDetected();
    }

    protected void setMxpGyroZeroYaw(vv_OpMode aOpMode) {
        baseMxpGyroSensor.zeroYaw();
    }


    class MotorNameNotKnownException extends Exception {

        MotorNameNotKnownException(String message) {
            super(message);
        }
    }

    class MotorStalledException extends Exception {

        MotorStalledException(String message) {
            super(message);
        }
    }


}