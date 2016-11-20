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
    // Encoder constants
	final static int TETRIX_MOTOR_ENCODER_COUNTS_PER_REVOLUTION = 1440;
	final static int ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION = 1120;

	final static float MOTOR_LOWER_POWER_THRESHOLD = 0.35f;
    final static float MOTOR_RAMP_POWER_LOWER_LIMIT = 0.20f;
    final static float MOTOR_RAMP_POWER_UPPER_LIMIT = 0.5f;
    final static float MOTOR_SLOW_START_THRESHOLD = 0.30f;

	final static int GYRO_ERROR = 8;


    // Mecanum wheel properties
    final static float MECCANUM_WHEEL_DIAMETER = 4f;   //in inches
    final static float MECCANUM_WHEEL_ENCODER_MARGIN = 20;

	final static float ANALOG_STICK_THRESHOLD = .25f;

    // Extremes for servo that pushes beacon buttons
    static final double BUTTON_SERVO_MAX_POS     =  0.8f;     // Maximum rotational position
    static final double BUTTON_SERVO_MIN_POS     =  0.45f;

    //distance between wheels left to right of the Robot in inches.
    final static float ROBOT_TRACK_DISTANCE = 15.6f; //adjusted from observation.
    //max time to wait in a tight loop, for example in robot turns or autonomous moves
    final static int MAX_MOTOR_LOOP_TIME = 5000;
    //max turn velocity for a motor, to be used in autonomous turns of the robot
    final static float MAX_ROBOT_TURN_MOTOR_VELOCITY = 0.5f;
    //min turn velocity for a motor, to be used in autonomous turns of the robot
    final static float MIN_ROBOT_TURN_MOTOR_VELOCITY = 0.15f;
    //gyro offset to address inertia and gyro lag
    final static int GYRO_OFFSET = 10;

    final static float TURN_POWER = 0.7f;
    final static float TOUCH_SENSE_POWER = 0.15f;

    //turn this one for debugging messages.

    final static boolean DEBUG = true;
    //time to flash DEBUG message on telemtry
    final static long DEBUG_MESSAGE_DISPLAY_TIME = 500;


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
