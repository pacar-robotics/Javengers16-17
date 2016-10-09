package org.firstinspires.ftc.teamcode;

/**
 * Created by thomas on 9/25/2016.
 */

public class vv_Constants
{

    final static int TETRIX_MOTOR_ENCODER_COUNTS_PER_REVOLUTION = 1440;
    final static int ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION = 1120;
    final static float MOTOR_LOWER_POWER_THRESHOLD = 0.35f;
    final static float MOTOR_SLOW_START_THRESHOLD = 0.30f;
    final static int GYRO_ERROR = 8;
    final static float MECCANUM_WHEEL_DIAMETER = 9.95f;   //in centimeters
    final static float MECCANUM_WHEEL_ENCODER_MARGIN = 20;
    final static float ANALOG_STICK_THRESHOLD = .25f;

    static final double BUTTON_SERVO_MAX_POS     =  0.8f;     // Maximum rotational position
    static final double BUTTON_SERVO_MIN_POS     =  0.45f;

    DirectionEnum Direction;
    TurnDirectionEnum TurnDirection;

    enum DirectionEnum
    {
        Forward, Backward, SidewaysLeft, SidewaysRight;
    }
    enum MotorEnum
    {
        frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    }
    enum TurnDirectionEnum
    {
        Clockwise, Counterclockwise;
    }
    enum ButtonEnum
    {
        Left, Right;
    }
    enum TouchSensorEnum
    {
        buttonSensor;
    }
}
