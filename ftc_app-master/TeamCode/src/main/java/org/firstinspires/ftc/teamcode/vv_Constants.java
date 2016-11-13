package org.firstinspires.ftc.teamcode;

/**
 * Created by thomas on 9/25/2016.
 */

/**
 * Class vv_Constants provides all constants that will be used
 * Ensures that all values are in sync
 * Also contains enumerations for direction, motors and beacon buttons
 * @author Krittika Negandhi
 * @author Thomas Chemmanoor
 * @author Rachael Chacko
 */
public class vv_Constants
{
    final static float ROBOT_TRACK_DISTANCE = 19.4f;

    final static int MAX_MOTOR_LOOP_TIME = 10000;
    final static float MAX_TURN_SPEED = 0.5f;
    final static float MIN_TURN_SPEED = 0.15f;

    final static int GYRO_ERROR_MARGIN = 10;

    // Encoder constants
	final static int TETRIX_MOTOR_ENCODER_COUNTS_PER_REVOLUTION = 1440;
	final static int ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION = 1120;

	final static float MOTOR_LOWER_POWER_THRESHOLD = 0.35f;
	final static float MOTOR_SLOW_START_THRESHOLD = 0.30f;

    final static int LUMINOSITY_MINIMUM_WHITE = 15;

    // Mecanum wheel properties
	final static float MECCANUM_WHEEL_DIAMETER = 9.95f;   //in centimeters
	final static float MECCANUM_WHEEL_ENCODER_MARGIN = 20;

	final static float ANALOG_STICK_THRESHOLD = .25f;

    // Extremes for servo that pushes beacon buttons
    static final double BUTTON_SERVO_MAX_POS = 0.9f;     // Maximum rotational position
    static final double BUTTON_SERVO_MIN_POS = 0.35f;

    DirectionEnum Direction;
    TurnDirectionEnum TurnDirection;

    // Direction of movement for autonomous
    enum DirectionEnum
    {
        Forward, Backward, SidewaysLeft, SidewaysRight
    }

    // Names of motors
    enum MotorEnum
    {
        frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, armMotor
    }

    // Direction of turning
    enum TurnDirectionEnum
    {
        Clockwise, Counterclockwise
    }

    // Left or right beacon button depending on color
    enum ButtonEnum
    {
        Left, Right
    }
    
    enum TouchSensorEnum
    {
        buttonSensor
    }

}
