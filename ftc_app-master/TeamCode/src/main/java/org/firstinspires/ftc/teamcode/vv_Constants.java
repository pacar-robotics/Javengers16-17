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
    final static float MOTOR_RAMP_POWER_UPPER_LIMIT = 0.8f;
    final static float MOTOR_SLOW_START_THRESHOLD = 0.30f;

    final static float COLOR_SENSOR_WHITE_LIMIT = 2.0f;

	final static int GYRO_ERROR = 8;


    // Mecanum wheel properties
    final static float MECCANUM_WHEEL_DIAMETER = 4f;   //in inches
    final static float MECCANUM_WHEEL_ENCODER_MARGIN = 20;

	final static float ANALOG_STICK_THRESHOLD = .25f;
    final static float TRIGGER_THRESHOLD = .25f;

    // Extremes for servo that pushes beacon buttons
    static final double BEACON_SERVO_MAX_POS = 0.8f;     // Maximum rotational position
    static final double BEACON_SERVO_MID_POS = 0.4f;
    static final double BUTTON_SERVO_MIN_POS     =  0.45f;

    // Extremes for servo that controls the gate for launch
    static final double LAUNCH_GATE_SERVO_OPEN = 0.0f;     // Maximum rotational position
    static final double LAUNCH_GATE_SERVO_CLOSED = 0.3f;


    //distance between wheels left to right of the Robot in inches.
    final static float ROBOT_TRACK_DISTANCE = 15.6f; //adjusted from observation.
    //max time to wait in a tight loop, for example in robot turns or autonomous moves
    final static int MAX_MOTOR_LOOP_TIME = 10000;
    //max turn velocity for a motor, to be used in autonomous turns of the robot
    final static float MAX_ROBOT_TURN_MOTOR_VELOCITY = 0.5f;
    //min turn velocity for a motor, to be used in autonomous turns of the robot
    final static float MIN_ROBOT_TURN_MOTOR_VELOCITY = 0.15f;
    //gyro offset to address inertia and gyro lag
    final static int GYRO_OFFSET = 10;

    final static float TURN_POWER = 0.7f;
    final static float TOUCH_SENSE_POWER = 0.2f;

    //turn this one for debugging messages.

    final static boolean DEBUG = false;
    //time to flash DEBUG message on telemtry
    final static long DEBUG_MESSAGE_DISPLAY_TIME = 10;

    //controls whether debug messages remain on screen while new ones are written
    final static boolean DEBUG_AUTO_CLEAR = false;

    //time to wait in stall check code
    final static int ENCODED_MOTOR_STALL_TIME_DELTA = 200;

    //encoder clicks to check for Andymark motors for stall
    final static int ENCODED_MOTOR_STALL_CLICKS_ANDYMARK = 10;
    //encoder clicks to check for Andymark motors for stall
    final static int ENCODED_MOTOR_STALL_CLICKS_TETRIX = 10;

    //power used to rotate intake
    final static float INTAKE_POWER = 06f;
    //define the motors as constants instead of enumerations for speed.


    final static int FRONT_LEFT_MOTOR = 0;
    final static int FRONT_RIGHT_MOTOR = 1;
    final static int BACK_LEFT_MOTOR = 2;
    final static int BACK_RIGHT_MOTOR = 3;
    final static int INTAKE_MOTOR = 4;
    final static int ARM_MOTOR = 5;
    final static int WORM_DRIVE_MOTOR = 6;
    final static int CAP_BALL_MOTOR = 7;


    final static int LAUNCH_POWER_INCREMENT = 500;

    final static int LAUNCH_POWER_POSITION_MIN = 0;
    final static int LAUNCH_POWER_POSITION_AUTONOMOUS = 3500;
    final static int LAUNCH_POWER_POSITION_MAX = 4400;
    final static int LAUNCH_POWER_POSITION_MID = 4000;

    final static float WORM_DRIVE_POWER = 0.8f;
    final static float WORM_DRIVE_DURATION_MAX = 3000;
    final static float WORM_DRIVE_ENCODER_MARGIN = 20;
    BeaconColorEnum BeaconColor;
    DirectionEnum Direction;
    TurnDirectionEnum TurnDirection;

    enum BeaconColorEnum {
        RED,
        BLUE,
        UNKNOWN
    }

    // Direction of movement for autonomous
    enum DirectionEnum
    {
        Forward, Backward, SidewaysLeft, SidewaysRight
    }

    //values that control the worm drive motor to adjust tension of the Launch arm

    // Direction of turning
    enum TurnDirectionEnum
    {
        Clockwise, Counterclockwise
    }
    enum TouchSensorEnum
    {
        buttonSensor
    }
    enum BeaconServoStateEnum {
        Left, Right, Neutral
    }
    //Cap Ball Lift States
    enum CapBallStateEnum {
        Rest, Scoring_Position
    }
    //Ball Collector States
    enum IntakeStateEnum {
        Off, In, Out
    }
    //Spring Motor States used to score in different positions
    //Position 1 = 1 Tile Away
    //Position 2 = 2 Tiles Away
    //Position 3 = 3 Tiles Away
    //Position 4 = 4 Tiles Away
    enum SpringPositionsEnum {
        Rest, Position1, Position2, Position3, Position4
    }


}
