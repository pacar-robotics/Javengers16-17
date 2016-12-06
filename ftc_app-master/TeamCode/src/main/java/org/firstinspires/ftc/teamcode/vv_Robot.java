package org.firstinspires.ftc.teamcode;


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
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor capBallLift = null;
    private DcMotor armMotor = null;
    private DcMotor intakeMotor = null;
    private DcMotor wormDriveMotor = null;

    private Servo beaconServo = null;
    private Servo launcherGateServo = null;

    private TouchSensor buttonSensor;
    private TouchSensor ts_springSensor;
    private ColorSensor cs;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();


    public vv_Constants.DirectionEnum Direction;
    public vv_Constants.TurnDirectionEnum TurnDirection;
    public vv_Constants.CapBallStateEnum CapBallState;
    public vv_Constants.BallCollectorStateEnum BallCollectorState;
    public vv_Constants.SpringPositionsEnum SpringPosition;
    public vv_Constants.DriverEnum CurrentDriver;

    public void init(HardwareMap ahwMap, vv_OpMode aOpMode) throws InterruptedException{

        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftMotor = hwMap.dcMotor.get("motor_front_left");
        frontRightMotor = hwMap.dcMotor.get("motor_front_right");
        backLeftMotor = hwMap.dcMotor.get("motor_back_left");
        backRightMotor = hwMap.dcMotor.get("motor_back_right");

        armMotor = hwMap.dcMotor.get("motor_arm");
        intakeMotor = hwMap.dcMotor.get("motor_intake");
        wormDriveMotor = hwMap.dcMotor.get("motor_worm");


        cs = hwMap.colorSensor.get("color_line_sensor");


        ts_springSensor = hwMap.touchSensor.get("touch_arm_sensor");

        beaconServo = hwMap.servo.get("button_servo");
        launcherGateServo = hwMap.servo.get("servo_launcher_gate");

        launcherGateServo.setPosition(vv_Constants.LAUNCH_GATE_SERVO_CLOSED);

        buttonSensor = hwMap.touchSensor.get("touch_button_sensor");

        beaconServo.setPosition(0.65);


        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to zero power
        stopMotors(aOpMode);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        wormDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (wormDriveMotor.getCurrentPosition() != 0)
        {
            //wait until resetted
        }

        wormDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set enumerations

        CapBallState = vv_Constants.CapBallStateEnum.Rest;
        BallCollectorState = vv_Constants.BallCollectorStateEnum.Off;
        SpringPosition = vv_Constants.SpringPositionsEnum.Rest;

        CurrentDriver = vv_Constants.DriverEnum.MainDriver;

    }

    /**
     * Moves the cap ball lift to a preset position
     *
     * @param anOp     an object of vv_OpMode
     * @param Position Encoder Position to move Cap Ball Lift to
     * @param Power    the power in which the motor runs
     */
    public void moveCapBallLift(vv_OpMode anOp, int Position, float Power) throws InterruptedException {
        moveMotorUsingEncoderLimits(anOp, capBallLift, Position, Power,
                vv_Constants.CAP_BALL_LIFT_MAX, vv_Constants.CAP_BALL_LIFT_MIN);
    }

    /**
     * Moves the spring motor to a preset position
     *
     * @param anOp     an object of vv_OpMode
     * @param Position Encoder Position to move spring motor to
     */
    public void moveWormDriveMotor(vv_OpMode anOp, int Position) throws InterruptedException {
        moveMotorUsingEncoderLimits(anOp, wormDriveMotor, Position, vv_Constants.WORM_DRIVE_MOTOR_POWER, vv_Constants.SPRING_MAX_LIMIT, vv_Constants.SPRING_MIN_LIMIT);
    }

    public void decrementLauncherArmPosition (vv_OpMode anOp) {
        //TODO: FINISH
    }

    /**
     * Sets power to inputted motor
     *
     * @param aOpMode   an object of vv_OpMode
     * @param motorEnum list of motors, determines what motor the power will be applied to
     * @param power     power that is applied to the motor
     */
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
            case intakeMotor:
                intakeMotor.setPower(-power);
                break;
            case wormDriveMotor:
                wormDriveMotor.setPower(power);
                break;
            case capBallLiftMotor:
                capBallLift.setPower(power);
        }
    }

    public void setMotorMode(vv_OpMode aOpMode, vv_Constants.MotorEnum motorEnum, DcMotor.RunMode runMode) {
        switch(motorEnum){
            case armMotor:
                armMotor.setMode(runMode);
                break;
            case wormDriveMotor:
                wormDriveMotor.setMode(runMode);
                break;
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

        }

        //TODO: Finish this method up
    }


    /**
     * Returns true if the Launcher Arm is at its limit
     *
     * @return armSensor.isPressed; whether to touch sensor at the limit is pressed
     */
    public boolean isArmAtLimit(vv_OpMode aOpMode) {
        return ts_springSensor.isPressed();
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
     * Runs robot to a specific position. Can be called by other, more specific methods to move
     * forwards and backwards or sideways.
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
     * Runs motors. Can be called by a more specific method to move forwards and backwards or
     * sideways.
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


    public void pushButton(vv_OpMode aOpMode, vv_Constants.BeaconServoStateEnum buttonEnum) {

        switch (buttonEnum) {

            case Left:
                beaconServo.setPosition(vv_Constants.BUTTON_SERVO_MAX_POS);
                break;

            case Right:
                beaconServo.setPosition(vv_Constants.BUTTON_SERVO_MIN_POS);
                break;
            case Neutral:
                beaconServo.setPosition(vv_Constants.BUTTON_SERVO_NEUTRAL_POS);
        }
    }


    public boolean getButtonTouchValue(vv_OpMode aOpMode) throws InterruptedException {
        return buttonSensor.isPressed();
    }

    public boolean getSpringTouchValue(vv_OpMode aOpMode) throws InterruptedException {
        return ts_springSensor.isPressed();
    }

    //TODO: P3 init for the spring
    public void moveTillSpringTouch(vv_OpMode aOpMode) throws InterruptedException {
        while (!getSpringTouchValue(aOpMode)) {
            wormDriveMotor.setPower(.4f);
        }
        wormDriveMotor.setPower(.0f);
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

    /**
     * Moves a motor to an Encoder Position by setting the target position, and waiting until the
     * motor reaches the target position
     *
     * @param anOp           an object of vv_OpMode
     * @param motor          The motor which will be run
     * @param targetPosition Encoder value in which the motor will be run to
     * @param power          the power which will be applied to the robot
     * @param maxEncoder     the top encoder limit, which is the encoder value when the lift is at
     *                       the top
     * @param minEncoder     the bottom encoder limit, which is the encoder value when the lift is
     *                       at rest; usually 0
     */
    public void moveMotorUsingEncoderLimits(vv_OpMode anOp, DcMotor motor, int targetPosition, float power, int maxEncoder, int minEncoder) throws InterruptedException {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(targetPosition);
        motor.setPower(Math.abs(power));

        //waits until the motor reaches the position
        while (motor.isBusy() && (Math.abs(motor.getCurrentPosition()) - Math.abs(targetPosition) >= vv_Constants.DC_MOTOR_ENCODER_MARGIN)
                && (Math.abs(motor.getCurrentPosition()) <= maxEncoder - vv_Constants.DC_MOTOR_ENCODER_MARGIN)
                && (Math.abs(motor.getCurrentPosition()) >= minEncoder)) {

            //runs till it approaches target position
        }

        motor.setPower(0);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void closeLauncherGate (vv_OpMode anOp) {
        launcherGateServo.setPosition(vv_Constants.LAUNCH_GATE_SERVO_CLOSED);
    }

    public void openLauncherGate (vv_OpMode anOp) {
        launcherGateServo.setPosition(vv_Constants.LAUNCH_GATE_SERVO_OPEN);
    }
}
