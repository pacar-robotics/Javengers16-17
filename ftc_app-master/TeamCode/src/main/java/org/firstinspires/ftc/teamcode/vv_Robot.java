package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by thomas on 9/25/2016.
 */

public class vv_Robot
{
	private DcMotor frontLeftMotor = null;
	private DcMotor frontRightMotor = null;
	private DcMotor backLeftMotor = null;
	private DcMotor backRightMotor = null;
	private DcMotor capBallLift = null;

	private Servo buttonServo = null;

	private TouchSensor buttonSensor;

	HardwareMap hwMap = null;
	private ElapsedTime period = new ElapsedTime();


	public void init(HardwareMap ahwMap)
	{
		// save reference to HW Map
		hwMap = ahwMap;

		// Define and Initialize Motors
		frontLeftMotor = hwMap.dcMotor.get("motor_front_left");
		frontRightMotor = hwMap.dcMotor.get("motor_front_right");
		backLeftMotor = hwMap.dcMotor.get("motor_back_left");
		backRightMotor = hwMap.dcMotor.get("motor_back_right");
		//TODO: Map Cap Ball Lift, and Direction, and mode


		buttonServo = hwMap.servo.get("button_servo");

		buttonSensor = hwMap.touchSensor.get("touch_button_sensor");

		buttonServo.setPosition(0.65);

		frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

		// Set all motors to zero power
		stopMotors();

		// Set all motors to run without encoders.
		// May want to use RUN_USING_ENCODERS if encoders are installed.

	}

    //moves cap ball lift to preset position
    public void moveCapBallLiftToPosition (vv_Constants.CapBallStateEnum capBallState)
    {
        switch (capBallState)
        {
            case Rest

        }
    }

	public void setPower(vv_Constants.MotorEnum motorEnum, float power)
	{
		switch (motorEnum)
		{
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
		}
	}

	public void setRobotMode(DcMotor.RunMode runMode)
	{
		frontLeftMotor.setMode(runMode);
		frontRightMotor.setMode(runMode);
		backLeftMotor.setMode(runMode);
		backRightMotor.setMode(runMode);
	}

	public void runRobotToPositionFB(vv_OpMode aOpMode, int position, float Power) throws InterruptedException
	{
		//using the generic method with all powers set to the same value and all positions set to the same position
		runRobotToPosition(aOpMode, Power, Power, Power, Power, position, position, position, position);
	}

	public void runRobotToPositionSideways(vv_OpMode aOpMode, int position, float Power) throws InterruptedException
	{
		//using the generic method with all powers set to the same value and all positions set to the same position
		runRobotToPosition(aOpMode, -Power, Power, Power, -Power, -position, position, position, -position);
	}


	public void runRobotToPosition(vv_OpMode aOpMode, float fl_Power, float fr_Power,
	                               float bl_Power, float br_Power, int fl_Position,
	                               int fr_Position, int bl_Position, int br_Position)
			throws InterruptedException
	{
		//reset motor encoders
		frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		while (frontLeftMotor.getCurrentPosition() != 0)
		{
			//wait until motors are reset
			Thread.sleep(20);
		}

		//sets all motors to run to a position
		setRobotMode(DcMotor.RunMode.RUN_TO_POSITION);

		//reset encoder for 1 wheel
		frontLeftMotor.setTargetPosition(fl_Position);
		frontRightMotor.setTargetPosition(fr_Position);
		backLeftMotor.setTargetPosition(bl_Position);
		backRightMotor.setTargetPosition(br_Position);

		//sets the the power of all motors
		setPower(vv_Constants.MotorEnum.frontLeftMotor, fl_Power);
		setPower(vv_Constants.MotorEnum.frontRightMotor, fr_Power);
		setPower(vv_Constants.MotorEnum.backLeftMotor, bl_Power);
		setPower(vv_Constants.MotorEnum.backRightMotor, br_Power);

		//wait until robot reaches target position
		//testing the wheels on the opposite sides of the robot because each might have a different position for sideways movements

		while ((Math.abs(frontLeftMotor.getCurrentPosition()) < Math.abs(fl_Position) - vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN) ||
				(Math.abs(frontRightMotor.getCurrentPosition()) < Math.abs(fr_Position) - vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN)
				|| (Math.abs(backRightMotor.getCurrentPosition()) < Math.abs(br_Position) - vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN) ||
				(Math.abs(backLeftMotor.getCurrentPosition()) < Math.abs(bl_Position) - vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN))
		{
			//report motor positions for debugging

			// TODO: UNCOMMENT THIS!!!!
			aOpMode.telemetryAddData("Motor FL", "Values", "" + frontLeftMotor.getCurrentPosition());
			aOpMode.telemetryAddData("Motor FR", "Values", "" + frontRightMotor.getCurrentPosition());
			aOpMode.telemetryAddData("Motor BL", "Values", "" + backLeftMotor.getCurrentPosition());
			aOpMode.telemetryAddData("Motor BR", "Values", "" + backRightMotor.getCurrentPosition());
			aOpMode.telemetryUpdate();

		}
		stopMotors();

		Thread.sleep(100);
	}


	public void runRobotToPositionWithAngle(vv_OpMode aOpMode, float fl_Power, float fr_Power,
	                                        float bl_Power, float br_Power, int fl_Position,
	                                        int fr_Position, int bl_Position, int br_Position, float angle)
			throws InterruptedException
	{

		//reset motor encoders
		frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		while (frontLeftMotor.getCurrentPosition() != 0)
		{
			//wait until motors are reset
			Thread.sleep(20);
		}

		//sets all motors to run to a position
		setRobotMode(DcMotor.RunMode.RUN_TO_POSITION);

		//reset encoder for 1 wheel
		frontLeftMotor.setTargetPosition(fl_Position);
		frontRightMotor.setTargetPosition(fr_Position);
		backLeftMotor.setTargetPosition(bl_Position);
		backRightMotor.setTargetPosition(br_Position);

		//sets the the power of all motors
		setPower(vv_Constants.MotorEnum.frontLeftMotor, fl_Power);
		setPower(vv_Constants.MotorEnum.frontRightMotor, fr_Power);
		setPower(vv_Constants.MotorEnum.backLeftMotor, bl_Power);
		setPower(vv_Constants.MotorEnum.backRightMotor, br_Power);

		//wait until robot reaches target position
		//testing the wheels on the opposite sides of the robot because each might have a different position for sideways movements

		while ((Math.abs(frontLeftMotor.getCurrentPosition()) < Math.abs(fl_Position) - vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN) ||
				(Math.abs(frontRightMotor.getCurrentPosition()) < Math.abs(fr_Position) - vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN)
				|| (Math.abs(backRightMotor.getCurrentPosition()) < Math.abs(br_Position) - vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN) ||
				(Math.abs(backLeftMotor.getCurrentPosition()) < Math.abs(bl_Position) - vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN))
		{
			//report motor positions for debugging
			aOpMode.telemetryAddData("Motor FL", "Values", "" + frontLeftMotor.getCurrentPosition());
			aOpMode.telemetryAddData("Motor FR", "Values", "" + frontRightMotor.getCurrentPosition());
			aOpMode.telemetryAddData("Motor BL", "Values", "" + backLeftMotor.getCurrentPosition());
			aOpMode.telemetryAddData("Motor BR", "Values", "" + backRightMotor.getCurrentPosition());
			aOpMode.telemetryUpdate();

		}
		stopMotors();

		Thread.sleep(100);
	}


	public void runMotorsFB(vv_OpMode aOpMode, float Power)
			throws InterruptedException
	{

		runMotors(aOpMode, Power, Power, Power, Power);
	}

	public void runMotorsSideways(vv_OpMode aOpMode, float Power)
			throws InterruptedException
	{

		runMotors(aOpMode, -Power, Power, Power, -Power);
	}


	public void runMotors(vv_OpMode aOpMode, float fl_Power, float fr_Power, float bl_Power, float br_Power)
			throws InterruptedException
	{

		frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		//sets the the power of all motors
		setPower(vv_Constants.MotorEnum.frontLeftMotor, fl_Power);
		setPower(vv_Constants.MotorEnum.frontRightMotor, fr_Power);
		setPower(vv_Constants.MotorEnum.backLeftMotor, bl_Power);
		setPower(vv_Constants.MotorEnum.backRightMotor, br_Power);
	}


	public void stopMotors()
	{
		frontLeftMotor.setPower(0);
		frontRightMotor.setPower(0);
		backLeftMotor.setPower(0);
		backRightMotor.setPower(0);
	}

	public void pushButton(vv_Constants.BeaconServoStateEnum beaconState)
	{

		switch (beaconState)
		{

			case Left:
				buttonServo.setPosition(vv_Constants.BUTTON_SERVO_LEFT_POS);
				break;

			case Right:
				buttonServo.setPosition(vv_Constants.BUTTON_SERVO_RIGHT_POS);
				break;
            case Neutral:
                buttonServo.setPosition(vv_Constants.BUTTON_SERVO_NEUTRAL_POS);
		}
	}

	public boolean getButtonTouchValue() throws InterruptedException
	{
		return buttonSensor.isPressed();
	}

	public void waitForTick(long periodMs) throws InterruptedException
	{

		long remaining = periodMs - (long) period.milliseconds();

		// sleep for the remaining portion of the regular cycle period.
		if (remaining > 0)
			Thread.sleep(remaining);

		// Reset the cycle clock for the next pass.
		period.reset();
	}

    //TODO: FINISH
    public void moveMotorUsingEncoderLimits(DcMotor motor, int targetPosition, float power, int maxEncoder, int minEncoder) throws InterruptedException
    {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(targetPosition);
        motor.setPower(Math.abs(power));

        while (motor.isBusy() || (Math.abs(motor.getCurrentPosition() - (targetPosition)) > vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN)
            && !(motor.getCurrentPosition() > Math.abs(maxEncoder - vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN)
                && !(motor.getCurrentPosition() < Math.abs(minEncoder - vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN)) {

        }
    }
}
