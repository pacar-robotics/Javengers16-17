package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.vv_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.vv_Constants.ButtonEnum;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.Backward;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.Forward;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysLeft;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysRight;
import static org.firstinspires.ftc.teamcode.vv_Constants.GYRO_OFFSET;
import static org.firstinspires.ftc.teamcode.vv_Constants.MAX_MOTOR_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.vv_Constants.MAX_ROBOT_TURN_MOTOR_VELOCITY;
import static org.firstinspires.ftc.teamcode.vv_Constants.MECCANUM_WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.vv_Constants.MIN_ROBOT_TURN_MOTOR_VELOCITY;
import static org.firstinspires.ftc.teamcode.vv_Constants.MotorEnum;
import static org.firstinspires.ftc.teamcode.vv_Constants.MotorEnum.armMotor;
import static org.firstinspires.ftc.teamcode.vv_Constants.ROBOT_TRACK_DISTANCE;
import static org.firstinspires.ftc.teamcode.vv_Constants.TOUCH_SENSE_POWER;
import static org.firstinspires.ftc.teamcode.vv_Constants.TURN_POWER;
import static org.firstinspires.ftc.teamcode.vv_Constants.TurnDirectionEnum;

/**
 * Created by thomas on 9/25/2016.
 */

public class vv_Lib {
    private vv_Robot robot;


    public vv_Lib(vv_OpMode aOpMode) throws InterruptedException{
        robot = new vv_Robot();
        robot.init(aOpMode, aOpMode.hardwareMap);
    }

    /**
     * moveWheels method
     * @param aOpMode - object of vv_OpMode class
     * @param distance - in centimeters
     * @param Power - float
     * @param Direction - forward, backward, sideways left, or sideways right
     * @throws InterruptedException
     */
    public void moveWheels(vv_OpMode aOpMode, float distance, float Power, DirectionEnum Direction)
            throws InterruptedException, vv_Robot.MotorNameNotKnownException {
        if (Direction == Forward) {
            // moving the robot forward
            moveForwardToPosition(aOpMode, distance, Power);
        } else if (Direction == Backward) {
            // moving the robot forward
            moveBackwardToPosition(aOpMode, distance, Power);
        } else if (Direction == SidewaysLeft) {
            // moving the robot forward
            moveSidewaysLeftToPosition(aOpMode, distance, Power);
        } else if (Direction == SidewaysRight) {
            // moving the robot forward
            moveSidewaysRightToPosition(aOpMode, distance, Power);
        }
        // code for moving forward, backward, sideways
    }

    public void setupShot(vv_OpMode aOpMode) throws InterruptedException, vv_Robot.MotorNameNotKnownException
    {


        robot.setMotorMode(aOpMode, armMotor, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(!robot.isArmAtLimit(aOpMode)){
            robot.setPower(aOpMode, armMotor, 1.0f);
        }
        robot.setPower(aOpMode, armMotor, 0.0f);

        Thread.sleep(100);

    }

    public void shootBall(vv_OpMode aOpMode) throws InterruptedException, vv_Robot.MotorNameNotKnownException
    {
        robot.setMotorMode(aOpMode, MotorEnum.armMotor, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.setPower(aOpMode, MotorEnum.armMotor, 1.0f);

        Thread.sleep(500);

        robot.setPower(aOpMode, MotorEnum.armMotor, 0.0f);
    }
    
    /**
     * Using encoders, this method turns the Robot clockwise or counter clockwise based on angle given.
     Calculates the turn distance by multiplying the angle by conversion factors to get to an encoder value
     *
     * @param aOpMode an object of the vv_OpMode class
     * @param power power in which to apply to each motor
     * @param angle angle in which the robot will turn to based on the current position as 0 degree
     * @param TurnDirection Turns either Clockwise or Counterclockwise
     * @throws InterruptedException
     */
    public void turnUsingEncoders(vv_OpMode aOpMode, float power, float angle, TurnDirectionEnum TurnDirection)
            throws InterruptedException, vv_Robot.MotorNameNotKnownException {

        //calculate the turn distance to be used in terms of encoder clicks.
        //for Andymark encoders.

        int turnDistance = (int) (2 * ((ROBOT_TRACK_DISTANCE) * angle
                * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION) /
                (MECCANUM_WHEEL_DIAMETER * 360));


        switch (TurnDirection) {
            case Clockwise:
                robot.runRobotToPosition(aOpMode, power, power, power, power, turnDistance, -turnDistance, turnDistance, -turnDistance);
                break;
            case Counterclockwise:
                robot.runRobotToPosition(aOpMode, power, power, power, power, -turnDistance, turnDistance, -turnDistance, turnDistance);
                break;
        }

        //wait just a bit for the commands to complete
        Thread.sleep(50);
    }

    public void pushAButton(vv_OpMode aOpMode, ButtonEnum buttonEnum) {
        robot.pushButton(aOpMode, buttonEnum);
    }

    public void turnUsingGyro(vv_OpMode aOpMode, float power, float angle, TurnDirectionEnum TurnDirection) {
        // do we need direction?
        // absolute vs. relative turns
    }


    public boolean senseTouch(vv_OpMode aOpMode) throws InterruptedException {
        return robot.getButtonTouchValue(aOpMode);
    }

    public void moveTillTouch(vv_OpMode aOpMode) throws InterruptedException, vv_Robot.MotorNameNotKnownException {
        aOpMode.reset_timer();
        while (!senseTouch(aOpMode) && aOpMode.time_elapsed() < MAX_MOTOR_LOOP_TIME) {
            robot.runMotors(aOpMode, TOUCH_SENSE_POWER, TOUCH_SENSE_POWER,
                    TOUCH_SENSE_POWER, TOUCH_SENSE_POWER);
            aOpMode.idle();
        }
        robot.stopBaseMotors(aOpMode);
        if (senseTouch(aOpMode)) {
            aOpMode.DBG("Touched Sensor");
        }
    }

    /**
     * Method that moves robot until the color white is detected
     * Used to stop at white line when going from first to second beacon
     * @param aOpMode - object of vv_OpMode class
     * @param cs
     * @throws InterruptedException
     */
    public void moveTillColor(vv_OpMode aOpMode, ColorSensor cs) throws InterruptedException, vv_Robot.MotorNameNotKnownException {
        while (!((cs.red() < 235) || (cs.green() < 235) || (cs.blue() < 235))) {
            aOpMode.telemetryAddFormattedData("test: ", "cs red value: ", cs.red());
            aOpMode.telemetryAddFormattedData("test1: ", "cs green value: ", cs.green());
            aOpMode.telemetryAddFormattedData("test2: ", "cs blue value: ", cs.blue());
            moveSidewaysLeft(aOpMode, .3f);
        }

    }


    public void showFloorColorSensorLumnosityOnTelemetry(vv_OpMode aOpMode, boolean updateTheDisplay) {

        aOpMode.telemetryAddData("Floor Sensor", "Luminosity",":"+robot.getFloorColorSensorAlpha(aOpMode));
        if (updateTheDisplay) {
            aOpMode.telemetryUpdate();
        }
    }

    public void showBaseGyroSensorHeadingOnTelemetry(vv_OpMode aOpMode, boolean updateTheDisplay) {
        aOpMode.telemetryAddData("Gyro Sensor", "Heading", ":" + robot.getBaseGyroSensorHeading(aOpMode));
        if (updateTheDisplay) {
            aOpMode.telemetryUpdate();
        }
    }


    public void showBaseGyroSensorIntegratedZValueOnTelemetry(vv_OpMode aOpMode, boolean updateTheDisplay) {

        aOpMode.telemetryAddData("Gyro Sensor", "Integrated Z", ":" + robot.getBaseGyroSensorIntegratedZValue(aOpMode));
        if (updateTheDisplay) {
            aOpMode.telemetryUpdate();
        }
    }


    public void turnFloorColorSensorLedOn(vv_OpMode aOpMode)throws InterruptedException{
        robot.enableFloorColorSensorLed(aOpMode);
    }

    public void turnFloorColorSensorLedOff(vv_OpMode aOpMode)throws InterruptedException{
        robot.disableFloorColorSensorLed(aOpMode);
    }

    //Moves robot forward with a distance supplied in centimeters and power between 0 and 1
    private void moveForwardToPosition(vv_OpMode aOpMode, float distance, float Power) throws InterruptedException, vv_Robot.MotorNameNotKnownException {
        //we need to store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = (int) ((distance / (Math.PI * MECCANUM_WHEEL_DIAMETER)) * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //runs the robot to position
        robot.runRobotToPositionFB(aOpMode, targetPosition, Power);
    }

    //Moves robot backward with a distance supplied in centimeters and power between 0 and 1
    private void moveBackwardToPosition(vv_OpMode aOpMode, float distance, float Power) throws InterruptedException, vv_Robot.MotorNameNotKnownException {
        //we need to store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = -(int) ((distance / (Math.PI * MECCANUM_WHEEL_DIAMETER)) * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //runs the robot to position with negative power
        robot.runRobotToPositionFB(aOpMode, -targetPosition, Power);
    }

    private void moveSidewaysLeftToPosition(vv_OpMode aOpMode, float distance, float Power) throws InterruptedException, vv_Robot.MotorNameNotKnownException {
        //we need to store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = (int) ((distance / (Math.PI * MECCANUM_WHEEL_DIAMETER)) * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //runs the robot to position with negative power
        robot.runRobotToPositionSideways(aOpMode, targetPosition, Power);
    }

    private void moveSidewaysRightToPosition(vv_OpMode aOpMode, float distance, float Power)
            throws InterruptedException, vv_Robot.MotorNameNotKnownException {
        //we need to store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = -(int) ((distance / (Math.PI * MECCANUM_WHEEL_DIAMETER)) * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //runs the robot to position with negative power
        robot.runRobotToPositionSideways(aOpMode, -targetPosition, Power);
    }


    //DO NOT USE THIS METHOD
    //IT IS NOT COMPLETED
    public void moveAtAngle(vv_OpMode aOpMode, double distance, float Power, float Angle)
            throws InterruptedException, vv_Robot.MotorNameNotKnownException {
        //we need to store the encoder target position
        int VldtargetPosition;
        int VrdtargetPosition;
        double Vld_distance = 0;
        double Vrd_distance = 0;

        float fl_Power = (float) ((Math.pow(Math.sin(Angle), 2.0) - (Math.pow(Math.cos(Angle), 2.0))));

        float fr_Power = (float) (-(Math.pow(Math.sin(Angle), 2.0) - (Math.pow(Math.cos(Angle), 2.0))));

        float bl_Power = (float) (-(Math.pow(Math.sin(Angle), 2.0) - (Math.pow(Math.cos(Angle), 2.0))));

        float br_Power = (float) ((Math.pow(Math.sin(Angle), 2.0) - (Math.pow(Math.cos(Angle), 2.0))));

        if (Angle > 0 && Angle < 45) {
            Angle = 45 - (45 % Angle);
            Vld_distance = 1;
            Vrd_distance = 1;
            Vld_distance *= ((Angle * distance) / Math.sin(90));
            Vrd_distance *= (((90 - Angle) * distance) / Math.sin(90));
        } else if (Angle > 45 && Angle < 90) {
            Angle %= 45;
            Vld_distance = -1;
            Vrd_distance = 1;
            Vld_distance *= ((Angle * distance) / Math.sin(90));
            Vrd_distance *= (((90 - Angle) * distance) / Math.sin(90));
        } else if (Angle > 90 && Angle < 135) {
            Angle = 45 - (45 % Angle);
            Vld_distance = -1;
            Vrd_distance = 1;
            Vrd_distance *= ((Angle * distance) / Math.sin(90));
            Vld_distance *= (((90 - Angle) * distance) / Math.sin(90));
        } else if (Angle > 135 && Angle < 180) {
            Angle %= 45;
            Vld_distance = -1;
            Vrd_distance = -1;
            Vrd_distance *= ((Angle * distance) / Math.sin(90));
            Vld_distance *= (((90 - Angle) * distance) / Math.sin(90));
        } else if (Angle > 180 && Angle < 225) {
            Angle = 45 - (45 % Angle);
            Vld_distance = -1;
            Vrd_distance = -1;
            Vld_distance *= ((Angle * distance) / Math.sin(90));
            Vrd_distance *= (((90 - Angle) * distance) / Math.sin(90));
        } else if (Angle > 225 && Angle < 270) {
            Angle %= 45;
            Vld_distance = 1;
            Vrd_distance = -1;
            Vld_distance *= ((Angle * distance) / Math.sin(90));
            Vrd_distance *= (((90 - Angle) * distance) / Math.sin(90));
        } else if (Angle > 270 && Angle < 315) {
            Angle = 45 - (45 % Angle);
            Vld_distance = 1;
            Vrd_distance = -1;
            Vrd_distance *= ((Angle * distance) / Math.sin(90));
            Vld_distance *= (((90 - Angle) * distance) / Math.sin(90));
        } else if (Angle > 315 && Angle < 360) {
            Angle %= 45;
            Vld_distance = 1;
            Vrd_distance = 1;
            Vrd_distance *= ((Angle * distance) / Math.sin(90));
            Vld_distance *= (((90 - Angle) * distance) / Math.sin(90));
        }


        VldtargetPosition = (int) ((Vld_distance / (Math.PI * MECCANUM_WHEEL_DIAMETER)) *
                ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        VrdtargetPosition = (int) ((Vrd_distance / (Math.PI * MECCANUM_WHEEL_DIAMETER)) *
                ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);


        //runs the robot to position with negative power
        robot.runRobotToPositionWithAngle(aOpMode, fl_Power * Power, fr_Power * Power, bl_Power * Power,
                br_Power * Power, VrdtargetPosition, VldtargetPosition, VldtargetPosition,
                VrdtargetPosition, Angle);
    }


    public void runAllMotors(vv_OpMode aOpMode, float FLPower, float FRPower, float BLPower, float BRPower)
            throws InterruptedException, vv_Robot.MotorNameNotKnownException {
        robot.runMotors(aOpMode, FLPower, FRPower, BLPower, BRPower);
    }

    public void stopAllMotors(vv_OpMode aOpMode) {
        robot.stopBaseMotors(aOpMode);
        //TODO: Stop additional motors that are not on the base of the Robot.
    }

    //Moves robot forward with a distance supplied in centimeters and power between 0 and 1
    public void moveForward(vv_OpMode aOpMode, float Power) throws InterruptedException, vv_Robot.MotorNameNotKnownException {
        robot.runMotorsFB(aOpMode, Power);
    }

    //Moves robot backward with a distance supplied in centimeters and power between 0 and 1
    public void moveBackward(vv_OpMode aOpMode, float Power) throws InterruptedException, vv_Robot.MotorNameNotKnownException {
        robot.runMotorsFB(aOpMode, -Power);
    }

    public void moveSidewaysLeft(vv_OpMode aOpMode, float Power) throws InterruptedException, vv_Robot.MotorNameNotKnownException {
        robot.runMotorsSideways(aOpMode, Power);
    }

    public void moveSidewaysRight(vv_OpMode aOpMode, float Power) throws InterruptedException, vv_Robot.MotorNameNotKnownException {
        robot.runMotorsSideways(aOpMode, -Power);
    }

    public void turnGyroDegrees(vv_OpMode aOpMode, int turnDegrees) throws InterruptedException, vv_Robot.MotorNameNotKnownException {

        //this code has some issues due to gyro read lag of approximately 250ms as reported.
        //This will cause the turns to be approximate and will generally overshoot.
        //only use this code if other methods dont work.
        //we will be trying this with another more advanced gyro sensor.


        float frontLeftMotorPower = 0;
        float frontRightMotorPower = 0;
        float backLeftMotorPower = 0;
        float backRightMotorPower = 0;


        //reset the ZIntegrator before the turn, delay is built in.
        robot.resetBaseGyroZIntegrator(aOpMode);
        //start a timer to ensure no stuck loops.
        long startTime = System.currentTimeMillis();
        int zValue = robot.getBaseGyroSensorIntegratedZValue(aOpMode);

        //adjust turnDegrees for gyro offset

        float turnSignum = Math.signum(turnDegrees);
        turnDegrees = Math.abs(turnDegrees) - GYRO_OFFSET;
        turnDegrees = turnDegrees * (int) turnSignum;


        while (Math.abs(zValue)
                < Math.abs(turnDegrees) && (System.currentTimeMillis() - startTime) < MAX_MOTOR_LOOP_TIME) {

            //calculate proportional power to be used in turn. This starts off being 1 and drops off as the turn completes.
            float turnPower = (Math.abs(turnDegrees) -
                    Math.abs(zValue)) / Math.abs(turnDegrees);

            //check for range of motor power values.

            if (turnPower < MIN_ROBOT_TURN_MOTOR_VELOCITY) {
                turnPower = MIN_ROBOT_TURN_MOTOR_VELOCITY;
            }

            if (turnPower > MAX_ROBOT_TURN_MOTOR_VELOCITY) {
                turnPower = MAX_ROBOT_TURN_MOTOR_VELOCITY;
            }

            //adjust for direction of turn by examining the sign of the turn degrees
            turnPower = turnPower * Math.signum(turnDegrees);

            //set the velocities to be used.

            frontLeftMotorPower = -1 * turnPower;
            frontRightMotorPower = turnPower;
            backLeftMotorPower = -1 * turnPower;
            backRightMotorPower = turnPower;

            //re-scan zValue
            zValue = robot.getBaseGyroSensorIntegratedZValue(aOpMode);
            if (Math.abs(zValue) >= Math.abs(turnDegrees)) {
                //break out of loop.
                break;
            }

            robot.runMotors(aOpMode, frontLeftMotorPower, frontRightMotorPower,
                    backLeftMotorPower, backRightMotorPower);

            //wait so we dont read faster than gyro can provide results.
            // aOpMode.telemetryAddData("Inside Turn","Degrees",":"+
            //     robot.getBaseGyroSensorIntegratedZValue(aOpMode));
            //aOpMode.telemetryAddData("Inside Turn","Power",":"+turnPower);
            //aOpMode.telemetryUpdate();
        }
        robot.stopBaseMotors(aOpMode);

        Thread.sleep(100);
        aOpMode.telemetryAddData("Post Turn Diag", "Degrees", ":" +
                robot.getBaseGyroSensorIntegratedZValue(aOpMode));
        aOpMode.telemetryUpdate();
        Thread.sleep(1000);


    }

    public void turnAbsoluteGyroDegrees(vv_OpMode aOpMode, float fieldReferenceDegrees) throws InterruptedException, vv_Robot.MotorNameNotKnownException {
        //clockwise is represented by clockwise numbers.
        //counterclockwise by negative angle numbers in degrees.
        //the fieldReferenceDegrees parameters measures degrees off the initial reference frame when the robot is started and the gyro is
        //calibrated.
        // >> IMPORTANT: This depends on the zIntegratedHeading not being altered by relative turns !!!

        //first take the absolute degrees and modulus down to 0 and 359.

        float targetDegrees = fieldReferenceDegrees % 360;

        //compare to the current gyro zIntegrated heading and store the result.
        //the Integrated zValue returned is negative for clockwise turns
        float turnDegrees = targetDegrees - (-1) * robot.getBaseGyroSensorIntegratedZValue(aOpMode);

        //make the turn using encoders

        aOpMode.telemetryAddData("targetDegrees", "Value",
                ":" + targetDegrees);
        aOpMode.telemetryAddData("Starting Z", "Value",
                ":" + robot.getBaseGyroSensorIntegratedZValue(aOpMode));
        aOpMode.telemetryAddData("Turn Degrees", "Value",
                ":" + turnDegrees);

        aOpMode.telemetryUpdate();

        turnUsingEncoders(aOpMode, TURN_POWER, Math.abs(turnDegrees),
                turnDegrees > 0 ? TurnDirectionEnum.Clockwise :
                        TurnDirectionEnum.Counterclockwise);


    }

    public void testMotor(vv_OpMode aOpMode, MotorEnum motorName, float power, int duration) throws InterruptedException, vv_Robot.MotorNameNotKnownException {
        aOpMode.DBG("in vvLib test Motor");
        robot.testMotor(aOpMode, motorName, power, duration);
    }

    public void testEncodedMotor(vv_OpMode aOpMode, MotorEnum motorName, float power, int duration, int targetPosition)
            throws InterruptedException, vv_Robot.MotorNameNotKnownException, vv_Robot.MotorStalledException {
        aOpMode.DBG("in testEncodedMotor");
        robot.testEncodedMotor(aOpMode, motorName, power, duration, targetPosition);
    }
}
