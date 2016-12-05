package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.vv_Constants.ARM_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_BLUE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_RED_THRESHOLD;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_LEFT;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_LOOK_FOR_COLOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_NEUTRAL;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_RIGHT;
import static org.firstinspires.ftc.teamcode.vv_Constants.DEBUG;
import static org.firstinspires.ftc.teamcode.vv_Constants.ENCODED_MOTOR_STALL_CLICKS_TETRIX;
import static org.firstinspires.ftc.teamcode.vv_Constants.ENCODED_MOTOR_STALL_TIME_DELTA;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.INTAKE_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.IntakeStateEnum.Off;
import static org.firstinspires.ftc.teamcode.vv_Constants.LAUNCH_GATE_SERVO_CLOSED;
import static org.firstinspires.ftc.teamcode.vv_Constants.LAUNCH_GATE_SERVO_OPEN;
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



    private Servo beaconServo = null;
    private Servo launcherGateServo = null;
    private TouchSensor beaconTouchSensor;
    private ColorSensor beaconColorSensor;
    private TouchSensor armSensor;
    private LightSensor floorLightSensor;
    private ModernRoboticsI2cGyro base_gyro_sensor;
    private UltrasonicSensor floorUltrasonicSensor;
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

        floorLightSensor = hwMap.lightSensor.get("floor_light_sensor");
        beaconTouchSensor = hwMap.touchSensor.get("beacon_touch_sensor");
        beaconColorSensor = hwMap.colorSensor.get("beacon_color_sensor");
        floorUltrasonicSensor = hwMap.ultrasonicSensor.get("floor_ultrasonic_sensor");

        //turn the LED on the floor color sensor off at the start.
        //used for compatibility with older SDK code.
        floorLightSensor.enableLed(false);
        //wait for it to turn off.
        Thread.sleep(300);

        beaconColorSensor.enableLed(false);
        //wait for it to turn off.
        Thread.sleep(300);

        aOpMode.DBG("before gyro calib");

        base_gyro_sensor = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("base_gyro_sensor");
        base_gyro_sensor.calibrate();
        Thread.sleep(100);
        while (base_gyro_sensor.isCalibrating()) {
            //wait for calibration completion
            Thread.sleep(50);
            aOpMode.idle();
        }
        aOpMode.DBG("after gyro calib");


        armSensor = hwMap.touchSensor.get("touch_arm_sensor");

        beaconServo = hwMap.servo.get("servo_beacon");

        //initialize to the middle position.
        beaconServo.setPosition(BEACON_SERVO_NEUTRAL);


        launcherGateServo = hwMap.servo.get("servo_launcher_gate");
        //initialize to the closed position
        launcherGateServo.setPosition(LAUNCH_GATE_SERVO_CLOSED);
        //wait for these servos to reach desired state
        Thread.sleep(100);


        //buttonServo.setPosition(0.65);

        aOpMode.DBG("before motor dir set");

        motorArray[FRONT_LEFT_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);

        motorArray[FRONT_RIGHT_MOTOR].setDirection(DcMotorSimple.Direction.REVERSE);

        motorArray[BACK_LEFT_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);

        motorArray[BACK_RIGHT_MOTOR].setDirection(DcMotorSimple.Direction.REVERSE);



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

    }

    public void setPower(vv_OpMode aOpMode, int motorName, float power)
            throws InterruptedException {

        motorArray[motorName].setPower(power);
    }

    public DcMotor.RunMode getMotorMode(vv_OpMode aOpMode, int motorName
    ) throws InterruptedException {

        return motorArray[motorName].getMode();

    }

    public void setMotorMode(vv_OpMode aOpMode, int motorName,
                             DcMotor.RunMode runMode)
            throws InterruptedException {

        motorArray[motorName].setMode(runMode);
    }

    public boolean isArmAtLimit(vv_OpMode aOpMode) {
        return armSensor.isPressed();
        //TODO: Finish this method up
    }

    public void runRobotToPositionFB(vv_OpMode aOpMode, int position, float Power)
            throws InterruptedException {
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
    public void runRobotToPositionSideways(vv_OpMode aOpMode, int position, float Power)
            throws InterruptedException {
        //using the generic method with all powers set to the same value and all positions set to the same position
        runRobotToPosition(aOpMode, Power, Power, Power, Power, -position, position, position, -position);
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


        //sets the the power of all motors
        //since we are ramping up, start at the lowest power allowed.
        setPower(aOpMode, FRONT_LEFT_MOTOR, MOTOR_LOWER_POWER_THRESHOLD);
        setPower(aOpMode, FRONT_RIGHT_MOTOR, MOTOR_LOWER_POWER_THRESHOLD);
        setPower(aOpMode, BACK_LEFT_MOTOR, MOTOR_LOWER_POWER_THRESHOLD);
        setPower(aOpMode, BACK_RIGHT_MOTOR, MOTOR_LOWER_POWER_THRESHOLD);


        aOpMode.reset_timer();
        while (baseMotorsAreBusy() && (aOpMode.time_elapsed() < MAX_MOTOR_LOOP_TIME) &&
                (Math.abs(Math.abs(motorArray[FRONT_LEFT_MOTOR].getCurrentPosition()) -
                        Math.abs(fl_Position)) > MECCANUM_WHEEL_ENCODER_MARGIN)) {
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

            //in this runmode, the power does not control direction but the sign of the target position does.

            motorArray[FRONT_LEFT_MOTOR].setPower(rampedPower);
            motorArray[FRONT_RIGHT_MOTOR].setPower(rampedPower);
            motorArray[BACK_LEFT_MOTOR].setPower(rampedPower);
            motorArray[BACK_RIGHT_MOTOR].setPower(rampedPower);


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
    public void runMotorsSidewaysRight(vv_OpMode aOpMode, float Power) throws InterruptedException {

        runMotors(aOpMode, Power, -Power, -Power, Power);
    }

    public void runMotorsSidewaysLeft(vv_OpMode aOpMode, float Power) throws InterruptedException {

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

    public void stopBaseMotors(vv_OpMode aOpMode) throws InterruptedException {
        motorArray[FRONT_LEFT_MOTOR].setPower(0);
        motorArray[FRONT_RIGHT_MOTOR].setPower(0);
        motorArray[BACK_LEFT_MOTOR].setPower(0);
        motorArray[BACK_RIGHT_MOTOR].setPower(0);
        Thread.sleep(100);
    }

    public void turnBeaconArm(vv_OpMode aOpMode, vv_Constants.BeaconServoStateEnum beaconArmEnum)
            throws InterruptedException {

        switch (beaconArmEnum) {

            case Left:
                beaconServo.setPosition(BEACON_SERVO_LEFT);
                break;

            case Right:
                beaconServo.setPosition(BEACON_SERVO_RIGHT);
                break;
            case Neutral:
                beaconServo.setPosition(BEACON_SERVO_NEUTRAL);
                break;
            case Look:
                beaconServo.setPosition(BEACON_SERVO_LOOK_FOR_COLOR);
                break;
        }
        Thread.sleep(200);
    }

    public boolean getButtonTouchValue(vv_OpMode aOpMode) throws InterruptedException {
        return beaconTouchSensor.isPressed();
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
    public void enableBeaconColorSensorLed(vv_OpMode aOpMode) throws InterruptedException {
        beaconColorSensor.enableLed(true);
        //wait for it to turn on.
        Thread.sleep(300);
    }

    //turn the color sensor LED on the floor of the robot off
    public void disableBeaconColorSensorLed(vv_OpMode aOPMode) throws InterruptedException {
        beaconColorSensor.enableLed(false);
        //wait for it to turn off.
        Thread.sleep(300);
    }


    public int getBeaconColorSensorAlpha(vv_OpMode aOpMode) throws InterruptedException {
        Thread.sleep(30);
        return beaconColorSensor.alpha();
    }

    public vv_Constants.BeaconColorEnum getBeaconColor(vv_OpMode aOpMode) throws InterruptedException {
        Thread.sleep(30);
        if (beaconColorSensor.red() > BEACON_RED_THRESHOLD) {
            return vv_Constants.BeaconColorEnum.RED;
        }
        if (beaconColorSensor.blue() > BEACON_BLUE_THRESHOLD) {
            return vv_Constants.BeaconColorEnum.BLUE;
        }
        return vv_Constants.BeaconColorEnum.UNKNOWN;


    }

/*
    public int getbeaconLightSensorGreen(vv_OpMode aOpMode) throws InterruptedException {
        Thread.sleep(30);
        return beaconLightSensor.green();
    }

    public int getbeaconLightSensorBlue(vv_OpMode aOpMode) throws InterruptedException {
        Thread.sleep(30);
        return beaconLightSensor.blue();
    }

*/

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

            int stallPositionStart = motorArray[WORM_DRIVE_MOTOR].getCurrentPosition();
            long stallTimeStart = aOpMode.time_elapsed();
            //wait till the run is complete or the time runs out.
            Thread.sleep(ENCODED_MOTOR_STALL_TIME_DELTA);
            //stall code
            int stallPositionDelta = Math.abs(Math.abs(motorArray[WORM_DRIVE_MOTOR].getCurrentPosition())
                    - Math.abs(stallPositionStart));
            long stallTimeDelta = aOpMode.time_elapsed() - stallTimeStart;
            float stallVelocity = ((stallPositionDelta * 1.0f) / stallTimeDelta);

            //TODO: Stall code must be tested!!
            if (stallVelocity < stallVelocityThreshold) {
                //motor stalling ?
                //stop motor first
                aOpMode.DBG("in stall code throw testEncodedMotor");
                motorArray[WORM_DRIVE_MOTOR].setPower(0.0f);
                //throw exception indicating the problem.
                if (DEBUG) {
                    aOpMode.telemetryAddData("Stall Data", "", "stallV:" + stallVelocity
                            + "stallT" + stallVelocityThreshold
                            + "stallTDelta" + stallTimeDelta
                            + "stallPDelta" + stallPositionDelta);
                    aOpMode.telemetryUpdate();
                }
                throw new MotorStalledException("MotorName" + ":WormDriveMotor", stallVelocity, stallVelocityThreshold);
            }
        }
        //stop the motor
        motorArray[WORM_DRIVE_MOTOR].setPower(0.0f);

    }

    public double getFloorLightIntensity(vv_OpMode aOpMode) {
        return floorLightSensor.getLightDetected();

    }

    public void openLauncherGate() throws InterruptedException {
        launcherGateServo.setPosition(LAUNCH_GATE_SERVO_OPEN);
        Thread.sleep(100);
    }

    public void closeLauncherGate() throws InterruptedException {
        launcherGateServo.setPosition(LAUNCH_GATE_SERVO_CLOSED);
        Thread.sleep(100);
    }

    public double getLauncherGateServoPosition(vv_OpMode aOpMode) {
        return launcherGateServo.getPosition();
    }

    public void setLauncherGateServoPosition(vv_OpMode aOpMode, double position) {
        launcherGateServo.setPosition(position);
    }

    public int getBeaconColorRedValue(vv_OpMode aOpMode) {
        return beaconColorSensor.red();
    }

    public int getBeaconColorGreenValue(vv_OpMode aOpMode) {
        return beaconColorSensor.green();
    }

    public int getBeaconColorBlueValue(vv_OpMode aOpMode) {
        return beaconColorSensor.blue();
    }



    public vv_Constants.IntakeStateEnum getIntakeState() {
        return IntakeState;
    }

    public void setIntakeState(vv_Constants.IntakeStateEnum IntakeStateValue) {
        IntakeState = IntakeStateValue;
    }

    public void setBeaconPosition(vv_OpMode aOpMode, double position)
            throws InterruptedException {
        beaconServo.setPosition(position);
        Thread.sleep(100);
    }

    public double getBeaconPosition(vv_OpMode aOpMode)
            throws InterruptedException {
        return beaconServo.getPosition();

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