package org.firstinspires.ftc.teamcode;

/**
 * Class vv_Constants provides all constants that will be used
 * Ensures that all values are in sync
 * Also contains enumerations for direction, motors and beacon buttons
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
   final static float ANALOG_STICK_THRESHOLD = .25f;


    //Beacon Servo Positions
    static final double BUTTON_SERVO_LEFT_POS = 0.8f;     // Maximum rotational position
    static final double BUTTON_SERVO_RIGHT_POS = 0.45f;
    static final double BUTTON_SERVO_NEUTRAL_POS = 0.0f; //TODO: CHANGE VALUE

    //CapBall Constants //TODO: CHANGE VALUES
    static final int CAP_BALL_LIFT_REST = 0;
    static final int CAP_BALL_LIFT_SCORE = 5000;
    static final int CAP_BALL_LIFT_MAX = 5050;
    static final int CAP_BALL_LIFT_MIN = 0;
    static final float CAP_BALL_SCORE_POWER_FACTOR = .5f; //TODO: CHANGE

    static final float BALL_COLLECTOR_POWER = .3f; //TODO: CHANGE

    //Robot Dimensions
    static final double ROBOT_TRACK = 0.0; //TODO: CHANGE VALUE


  // Direction of turning

  
 

    // Direction of movement for autonomous
    enum DirectionEnum
    {
        Forward, Backward, SidewaysLeft, SidewaysRight;
    }
    DirectionEnum Direction;

 
   // Names of motors
    enum MotorEnum
    {
        frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, armMotor;
    }


  // Direction of turning
    enum TurnDirectionEnum
    {
        Clockwise, Counterclockwise;
    }
      TurnDirectionEnum TurnDirection;


    // Left or right beacon button depending on color
    enum ButtonEnum
    {
        Left, Right;
    }

    enum TouchSensorEnum {
        buttonSensor;
    }

    enum BeaconServoStateEnum {
        Left, Right, Neutral
    }

    enum CapBallStateEnum {
        Rest, Scoring_Position
    }

    enum BallCollectorStateEnum {
        Off, Intake, Outtake
    }

}
