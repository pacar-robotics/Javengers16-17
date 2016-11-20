package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by thomas on 9/25/2016.
 */

public class vv_Robot {
    HardwareMap hwMap = null;
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor armMotor = null;
    private Servo buttonServo = null;
    private TouchSensor buttonSensor;
    private TouchSensor armSensor;
    private ColorSensor cs;
    private ModernRoboticsI2cGyro base_gyro_sensor;
    private ElapsedTime period = new ElapsedTime();


    public void init(vv_OpMode aOpMode, HardwareMap ahwMap) throws InterruptedException{
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftMotor = hwMap.dcMotor.get("motor_front_left");
        frontRightMotor = hwMap.dcMotor.get("motor_front_right");
        backLeftMotor = hwMap.dcMotor.get("motor_back_left");
        backRightMotor = hwMap.dcMotor.get("motor_back_right");
        armMotor = hwMap.dcMotor.get("motor_arm");

        cs = hwMap.colorSensor.get("color_line_sensor");

        //turn the LED on the floor color sensor off at the start.
        //used for compatibility with older SDK code.
        cs.enableLed(false);
        //wait for it to turn off.

        Thread.sleep(300);

        base_gyro_sensor = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("base_gyro_sensor");
        base_gyro_sensor.calibrate();
        while (base_gyro_sensor.isCalibrating()) {
            //wait for calibration completion
            Thread.sleep(50);
            aOpMode.idle();
        }

        Thread.sleep(2000);


        armSensor = hwMap.touchSensor.get("touch_arm_sensor");

        buttonServo = hwMap.servo.get("button_servo");

        buttonSensor = hwMap.touchSensor.get("touch_button_sensor");

        buttonServo.setPosition(0.65);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all base motors to zero power
        stopBaseMotors(aOpMode);

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

    public DcMotor.RunMode getMotorMode(vv_OpMode aOpMode, vv_Constants.MotorEnum motorEnum
    ) throws MotorNameNotKnownException {


        switch (motorEnum) {
            case frontLeftMotor:
                return frontLeftMotor.getMode();
            case frontRightMotor:
                return frontRightMotor.getMode();
            case backLeftMotor:
                return backLeftMotor.getMode();
            case backRightMotor:
                return backRightMotor.getMode();
            case armMotor:
                return armMotor.getMode();
            default:
                throw new MotorNameNotKnownException("Motor Name Not Known:" + motorEnum.toString());

        }

    }

    public void setMotorMode(vv_OpMode aOpMode, vv_Constants.MotorEnum motorEnum,
                             DcMotor.RunMode runMode) throws MotorNameNotKnownException {

        //TODO: Finish this emthod        switch (motorEnum) {
        switch (motorEnum) {
            case frontLeftMotor:
                frontLeftMotor.setMode(runMode);
                break;
            case frontRightMotor:
                frontRightMotor.setMode(runMode);
                break;
            case backLeftMotor:
                backLeftMotor.setMode(runMode);
                break;
            case backRightMotor:
                backRightMotor.setMode(runMode);
                break;
            case armMotor:
                armMotor.setMode(runMode);
                break;
            default:
                throw new MotorNameNotKnownException("Motor Name Not Known:" + motorEnum.toString());

        }

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

        //save the current run mode
        DcMotor.RunMode oldRunMode = frontLeftMotor.getMode();

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
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //reset encoder for 1 wheel
        frontLeftMotor.setTargetPosition(fl_Position);
        frontRightMotor.setTargetPosition(fr_Position);
        backLeftMotor.setTargetPosition(bl_Position);
        backRightMotor.setTargetPosition(br_Position);


        //sets the the power of all motors
        //since we are ramping up, start at the lowest power allowed.
        setPower(aOpMode, vv_Constants.MotorEnum.frontLeftMotor, vv_Constants.MOTOR_LOWER_POWER_THRESHOLD);
        setPower(aOpMode, vv_Constants.MotorEnum.frontRightMotor, vv_Constants.MOTOR_LOWER_POWER_THRESHOLD);
        setPower(aOpMode, vv_Constants.MotorEnum.backLeftMotor, vv_Constants.MOTOR_LOWER_POWER_THRESHOLD);
        setPower(aOpMode, vv_Constants.MotorEnum.backRightMotor, vv_Constants.MOTOR_LOWER_POWER_THRESHOLD);

        //wait until robot reaches target position


        aOpMode.reset_timer();
        while (baseMotorsAreBusy() && (aOpMode.time_elapsed() < vv_Constants.MAX_MOTOR_LOOP_TIME)) {
            //wait until motors havce completed movement or timed out.
            //report motor positions for debugging

            //adjust the motor speeds by adjusting Power proportional to distance that needs to be travelled.

            //
            //Ramped Move block formula:
            //RP=PMax(1-4*(0.5-DT/DD)^2)
            //where RP=Ramped Power, PMax is maximum power available, DT=Distance Travelled, DD=Distance to be travelled
            //fl_position (target for the front left motor in encoder clicks can be taken as the proxy for all motors.
            float rampedPowerRaw = (float) (vv_Constants.MOTOR_RAMP_POWER_UPPER_LIMIT * (1 - 4 * (Math.pow((0.5f -
                    Math.abs((frontLeftMotor.getCurrentPosition() * 1.0f) / fl_Position)), 2.0f))));

            //use another variable to check and adjust power limits, so we can display raw power values.
            float rampedPower = rampedPowerRaw;

            //check for upper and lower limits.
            if (rampedPower > vv_Constants.MOTOR_RAMP_POWER_UPPER_LIMIT) {
                rampedPower = vv_Constants.MOTOR_RAMP_POWER_UPPER_LIMIT;
            }
            if (rampedPower < vv_Constants.MOTOR_RAMP_POWER_LOWER_LIMIT) {
                rampedPower = vv_Constants.MOTOR_RAMP_POWER_LOWER_LIMIT;
            }

            //apply the new power values.
            //sets the the power of all motors

            setPower(aOpMode, vv_Constants.MotorEnum.frontLeftMotor, rampedPower);
            setPower(aOpMode, vv_Constants.MotorEnum.frontRightMotor, rampedPower);
            setPower(aOpMode, vv_Constants.MotorEnum.backLeftMotor, rampedPower);
            setPower(aOpMode, vv_Constants.MotorEnum.backRightMotor, rampedPower);



            // TODO: UNCOMMENT THIS!!!!
            if (vv_Constants.DEBUG) {
                aOpMode.telemetryAddData("Motor FL", "Values", ":" + frontLeftMotor.getCurrentPosition());
                aOpMode.telemetryAddData("Motor FR", "Values", ":" + frontRightMotor.getCurrentPosition());
                aOpMode.telemetryAddData("Motor BL", "Values", ":" + backLeftMotor.getCurrentPosition());
                aOpMode.telemetryAddData("Motor BR", "Values", ":" + backRightMotor.getCurrentPosition());
                aOpMode.telemetryAddData("Raw Ramped Power", "Values", ":" + rampedPowerRaw);
                aOpMode.telemetryAddData("Ramped Power ", "Values", ":" + rampedPower);
                aOpMode.telemetryAddData("fl_position", "Values", ":" + fl_Position);
                aOpMode.telemetryAddData("DTraveled/DTarget", "Values", ":" + Math.abs(frontLeftMotor.getCurrentPosition() / fl_Position));
                aOpMode.telemetryAddData("Squared Values", "Values", ":" +
                        Math.pow((0.5f - Math.abs((frontLeftMotor.getCurrentPosition() * 1.0f) / fl_Position)), 2.0f));


                aOpMode.telemetryUpdate();
            }
            aOpMode.idle();
        }
        stopBaseMotors(aOpMode);

        //restore old run modes
        frontLeftMotor.setMode(oldRunMode);
        frontRightMotor.setMode(oldRunMode);
        backLeftMotor.setMode(oldRunMode);
        backRightMotor.setMode(oldRunMode);

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
        try {
            setMotorMode(aOpMode, vv_Constants.MotorEnum.frontLeftMotor, DcMotor.RunMode.RUN_TO_POSITION);
            setMotorMode(aOpMode, vv_Constants.MotorEnum.frontRightMotor, DcMotor.RunMode.RUN_TO_POSITION);
            setMotorMode(aOpMode, vv_Constants.MotorEnum.backLeftMotor, DcMotor.RunMode.RUN_TO_POSITION);
            setMotorMode(aOpMode, vv_Constants.MotorEnum.backRightMotor, DcMotor.RunMode.RUN_TO_POSITION);
            while (frontLeftMotor.isBusy() || frontRightMotor.isBusy() ||
                    backLeftMotor.isBusy() || backRightMotor.isBusy()) {
                //wait until the motors have finished these tasks
                aOpMode.idle();
            }
        } catch (MotorNameNotKnownException mNNKE) {
            aOpMode.telemetryAddData("Motor Control Error", "Error", mNNKE.getMessage());
        }
        //reset encoders
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

        long startTime = System.currentTimeMillis();
        while (baseMotorsAreBusy() && ((System.currentTimeMillis() - startTime)) < vv_Constants.MAX_MOTOR_LOOP_TIME) {
            //wait while motors reach targets or we time out.
            //report motor positions for debugging
            aOpMode.telemetryAddData("Motor FL", "Values", "" + frontLeftMotor.getCurrentPosition());
            aOpMode.telemetryAddData("Motor FR", "Values", "" + frontRightMotor.getCurrentPosition());
            aOpMode.telemetryAddData("Motor BL", "Values", "" + backLeftMotor.getCurrentPosition());
            aOpMode.telemetryAddData("Motor BR", "Values", "" + backRightMotor.getCurrentPosition());
            aOpMode.telemetryUpdate();
            aOpMode.idle();

        }
        stopBaseMotors(aOpMode);
        //final value display
        aOpMode.telemetryAddData("Motor FL", "Values", "" + frontLeftMotor.getCurrentPosition());
        aOpMode.telemetryAddData("Motor FR", "Values", "" + frontRightMotor.getCurrentPosition());
        aOpMode.telemetryAddData("Motor BL", "Values", "" + backLeftMotor.getCurrentPosition());
        aOpMode.telemetryAddData("Motor BR", "Values", "" + backRightMotor.getCurrentPosition());
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

    public void stopBaseMotors(vv_OpMode aOpMode) {
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
        return (frontLeftMotor.isBusy() && frontRightMotor.isBusy() &&
                backLeftMotor.isBusy() && backRightMotor.isBusy());
    }

    class MotorNameNotKnownException extends Exception {
        MotorNameNotKnownException(String message) {
            super(message);
        }
    }


}