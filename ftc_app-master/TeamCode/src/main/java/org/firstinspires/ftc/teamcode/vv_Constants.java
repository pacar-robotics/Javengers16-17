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
    final static float MATRIX_MOTOR_ENCODER_COUNTS_PER_REVOLUTION = 1478.4f;

    final static int ARM_MOTOR_ENCODER_COUNTS_PER_REVOLUTION =
            ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION * 3 / 2;  //Gear ratio for Choo-Choo

    final static float MOTOR_LOWER_POWER_THRESHOLD = 0.20f;

    //forward/backward power limits
    final static float MOTOR_RAMP_FB_POWER_LOWER_LIMIT = 0.3f;
    final static float MOTOR_RAMP_FB_POWER_UPPER_LIMIT = 0.78f;

    //sideways power limit
    final static float MOTOR_RAMP_SIDEWAYS_POWER_LOWER_LIMIT = 0.6f;
    final static float MOTOR_RAMP_SIDEWAYS_POWER_UPPER_LIMIT = 0.78f;

    //sideways ultrasonic travel power limit
    final static float MOTOR_ULTRASONIC_SIDEWAYS_POWER_LOWER_LIMIT = 0.18f;
    final static float MOTOR_ULTRASONIC_SIDEWAYS_POWER_UPPER_LIMIT = 0.78f;


    final static float MOTOR_SLOW_START_THRESHOLD = 0.30f;

    final static float COLOR_SENSOR_WHITE_LIMIT = 2.0f;

	final static int GYRO_ERROR = 8;


    // Mecanum wheel properties
    final static float MECCANUM_WHEEL_DIAMETER = 4f;   //in inches
    final static float MECCANUM_WHEEL_ENCODER_MARGIN = 10;
    final static float MECCANUM_WHEEL_SIDE_TRACK_DISTANCE = 13.0f;
    final static float MECCANUM_WHEEL_FRONT_TRACK_DISTANCE = 14.5f;

    final static float STANDARD_DRIVE_POWER_FACTOR = 0.7f;

	final static float ANALOG_STICK_THRESHOLD = .25f;
    final static float TRIGGER_THRESHOLD = .10f;

    // Extremes for servo that pushes beacon buttons
    static final double BEACON_SERVO_LEFT_REST = 0.65f;     // Maximum rotational position
    static final double BEACON_SERVO_RIGHT_REST = 0.65f;
    static final double BEACON_SERVO_LEFT_PRESSED = 0.15f;
    static final double BEACON_SERVO_RIGHT_PRESSED = 0.23f;

    // Extremes for servo that controls the gate for launch
    static final double LAUNCH_FRONT_GATE_SERVO_OPEN = 0.8f;     // Maximum rotational position
    static final double LAUNCH_FRONT_GATE_SERVO_CLOSED = 0.45f;

    static final double LAUNCH_REAR_GATE_SERVO_OPEN = 0.7f;     // Maximum rotational position
    static final double LAUNCH_REAR_GATE_SERVO_CLOSED = 0.1f;

    static final double BALL_FLAG_SERVO_LOWERED = 0.0f;
    static final double BALL_FLAG_SERVO_RAISED = 0.8f;
    static final double BALL_FLAG_SERVO_ALARM = 0.1f;

    static final double CAP_BALL_SERVO_SECURED = 0.75f;
    static final double CAP_BALL_SERVO_RELEASED = 0.2f;



    //distance between wheels left to right of the Robot in inches.
    final static float ROBOT_TRACK_DISTANCE = 13.7f; //adjusted from observation.
    //max time to wait in a tight loop, for example in robot turns or autonomous moves
    final static int MAX_MOTOR_LOOP_TIME = 10000;
    //max turn velocity for a motor, to be used in autonomous turns of the robot
    final static float MAX_ROBOT_TURN_MOTOR_VELOCITY = 0.78f;
    //min turn velocity for a motor, to be used in autonomous turns of the robot
    final static float MIN_ROBOT_TURN_MOTOR_VELOCITY = 0.15f;
    //gyro offset to address inertia and gyro lag
    final static int GYRO_OFFSET = 10;

    final static float TURN_POWER = 0.95f;
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
    final static float INTAKE_POWER = 0.8f;
    //define the motors as constants instead of enumerations for speed.

    //power used to power cap ball movements
    final static float CAP_BALL_POWER = 0.8f;
    //define the motors as constants instead of enumerations for speed.

    final static int CAP_BALL_DURATION_MAX = 17000;

    final static int CAP_BALL_ENCODER_MARGIN = 50;

    final static int ARM_MOTOR_ENCODER_MARGIN = 50;

    //index of motors
    final static int FRONT_LEFT_MOTOR = 0;
    final static int FRONT_RIGHT_MOTOR = 1;
    final static int BACK_LEFT_MOTOR = 2;
    final static int BACK_RIGHT_MOTOR = 3;
    final static int INTAKE_MOTOR = 4;
    final static int ARM_MOTOR = 5;
    final static int WORM_DRIVE_MOTOR = 6;
    final static int CAP_BALL_MOTOR = 7;

    final static float LEFT_MOTOR_TRIM_FACTOR = 0.95f;
    final static float RIGHT_MOTOR_TRIM_FACTOR = 1.0f;

    //index of servos
    final static int LEFT_BEACON_BUTTON_SERVO = 0;
    final static int RIGHT_BEACON_BUTTON_SERVO = 1;


    final static int LAUNCH_POWER_INCREMENT = 75;

    final static int CAP_BALL_POSITION_INCREMENT = 5000;

    // Autonomous position = 425
    // Max position = 575
    // close position = -135

    final static int LAUNCH_POWER_POSITION_REST = 0;
    final static int LAUNCH_POWER_POSITION_AUTONOMOUS = 425;
    final static int LAUNCH_POWER_POSITION_MAX = 575;
    final static int LAUNCH_POWER_POSITION_MIN = -135;

    final static int PRE_INIT_LAUNCH_POSITION_INCREMENT = 200;
    final static float PRE_INIT_LAUNCH_POWER = 0.5f;

    final static float WORM_DRIVE_POWER = 0.8f;
    final static float WORM_DRIVE_DURATION_MAX = 3000;
    final static float WORM_DRIVE_ENCODER_MARGIN = 20;
    final static int BEACON_RED_THRESHOLD = 0;
    final static int BEACON_BLUE_THRESHOLD = 0;
    // Average of values is .26f. Half on, Half off is .23f to .24f
    final static float FLOOR_WHITE_THRESHOLD = 0.26f; //may need to calibrate
    final static boolean TEAM_RED = false;


    final static double EOPD_PROXIMITY_THRESHOLD = 0.00d;
    final static double RANGESENSOR_ULTRASONIC_PROXIMITY_THRESHOLD = 1.2d;
    final static double RANGESENSOR_OPTICAL_PROXIMITY_THRESHOLD = 1.0d;

    final static int CAP_BALL_ENCODER_UPPER_LIMIT = Math.round((45000 /
            TETRIX_MOTOR_ENCODER_COUNTS_PER_REVOLUTION) *
            MATRIX_MOTOR_ENCODER_COUNTS_PER_REVOLUTION); //for matrix
    //motor adjustments.


    BeaconColorEnum BeaconColor;
    DirectionEnum Direction;
    TurnDirectionEnum TurnDirection;

    enum BeaconColorEnum {
        RED,
        BLUE,
        UNKNOWN
    }

    enum AllianceColorEnum {
        BLUE,
        RED
    }

    //values that control the worm drive motor to adjust tension of the Launch arm

    // Direction of movement for autonomous
    enum DirectionEnum
    {
        Forward, Backward, SidewaysLeft, SidewaysRight
    }
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
        Left, Right, Neutral, Look
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
