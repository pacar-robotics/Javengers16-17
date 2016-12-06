package org.firstinspires.ftc.teamcode;

/**
 * Class vv_Constants provides all constants that will be used Ensures that all values are in sync
 * Also contains enumerations for direction, motors and beacon buttons
 *
 * @author Krittika Negandhi
 * @author Thomas Chemmanoor
 * @author Rachael Chacko
 */
public class vv_Constants {

    // Encoder constants
    final static int TETRIX_MOTOR_ENCODER_COUNTS_PER_REVOLUTION = 1440;
    final static int ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION = 1120;

    final static float MOTOR_LOWER_POWER_THRESHOLD = 0.35f;
    final static float MOTOR_SLOW_START_THRESHOLD = 0.30f;

    final static int GYRO_ERROR = 8;


    // Mecanum wheel properties
    final static float MECCANUM_WHEEL_DIAMETER = 9.95f;   //in centimeters
    final static float MECCANUM_WHEEL_ENCODER_MARGIN = 20;

    final static float DC_MOTOR_ENCODER_MARGIN = 20;

    final static float ANALOG_STICK_THRESHOLD = .25f;

    //Beacon Servo Positions
    static final double BUTTON_SERVO_MIN_POS = 0.8f;     // Maximum rotational position
    static final double BUTTON_SERVO_MAX_POS = 0.45f;
    static final double BUTTON_SERVO_NEUTRAL_POS = 0.625f;

    //CapBall Constants //TODO: CHANGE VALUES
    static final int CAP_BALL_LIFT_REST = 0;
    static final int CAP_BALL_LIFT_SCORE = 5000;
    static final int CAP_BALL_LIFT_MAX = 5050;
    static final int CAP_BALL_LIFT_MIN = 0;
    static final float CAP_BALL_SCORE_POWER_FACTOR = .5f; //TODO: CHANGE

    //Spring Motor Constants //TODO: CHANGE VALUES
    static final int SPRING_REST_POSITION = 0; //encoder value for the spring's starting position
    static final int SPRING_POSITION1 = 200; //encoder value for spring in order to shoot 1 tile away
    static final int SPRING_POSITION2 = 300; //encoder value for spring in order to shoot 2 tile away
    static final int SPRING_POSITION3 = 400; //encoder value for spring in order to shoot 3 tile away
    static final float WORM_DRIVE_MOTOR_POWER = .3f;
    static final int SPRING_MAX_LIMIT = 500;
    static final int SPRING_MIN_LIMIT = 0;

    //Ball Collector Constants
    static final float BALL_COLLECTOR_POWER = .3f; //TODO: CHANGE

    //Robot Dimensions
    static final double ROBOT_TRACK = 0.0; //TODO: CHANGE VALUE


    // Direction of movement for autonomous
    enum DirectionEnum {
        Forward, Backward, SidewaysLeft, SidewaysRight;
    }

    // Names of motors
    enum MotorEnum {
        frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, armMotor,
        capBallLiftMotor, intakeMotor, wormDriveMotor;
    }

    // Direction of turning
    enum TurnDirectionEnum {
        Clockwise, Counterclockwise;
    }

    enum TouchSensorEnum {
        buttonSensor, armSensor;
    }

    // Left or right beacon button depending on color
    enum BeaconServoStateEnum {
        Left, Right, Neutral
    }

    //Cap Ball Lift States
    enum CapBallStateEnum {
        Rest, Scoring_Position
    }

    //Ball Collector States
    enum BallCollectorStateEnum {
        Off, Intake, Outtake
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
