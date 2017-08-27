package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.io.IOException;

import static org.firstinspires.ftc.teamcode.vv_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.vv_Constants.ARM_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BALL_FLAG_SERVO_ALARM;
import static org.firstinspires.ftc.teamcode.vv_Constants.BALL_FLAG_SERVO_LOWERED;
import static org.firstinspires.ftc.teamcode.vv_Constants.BALL_FLAG_SERVO_RAISED;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_LEFT_PRESSED;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_LEFT_REST;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_RIGHT_PRESSED;
import static org.firstinspires.ftc.teamcode.vv_Constants.BEACON_SERVO_RIGHT_REST;
import static org.firstinspires.ftc.teamcode.vv_Constants.DEBUG;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.Backward;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.Forward;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysLeft;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysRight;
import static org.firstinspires.ftc.teamcode.vv_Constants.EOPD_PROXIMITY_THRESHOLD;
import static org.firstinspires.ftc.teamcode.vv_Constants.FLOOR_WHITE_MARGIN;
import static org.firstinspires.ftc.teamcode.vv_Constants.FLOOR_WHITE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.GYRO_OFFSET;
import static org.firstinspires.ftc.teamcode.vv_Constants.INTAKE_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.LAUNCH_POWER_INCREMENT;
import static org.firstinspires.ftc.teamcode.vv_Constants.LAUNCH_POWER_POSITION_MAX;
import static org.firstinspires.ftc.teamcode.vv_Constants.LAUNCH_POWER_POSITION_MIN;
import static org.firstinspires.ftc.teamcode.vv_Constants.LEFT_BEACON_BUTTON_SERVO;
import static org.firstinspires.ftc.teamcode.vv_Constants.MAX_MOTOR_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.vv_Constants.MAX_ROBOT_TURN_MOTOR_VELOCITY;
import static org.firstinspires.ftc.teamcode.vv_Constants.MECCANUM_WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.vv_Constants.MIN_ROBOT_TURN_MOTOR_VELOCITY;
import static org.firstinspires.ftc.teamcode.vv_Constants.MOTOR_ULTRASONIC_SIDEWAYS_POWER_LOWER_LIMIT;
import static org.firstinspires.ftc.teamcode.vv_Constants.MOTOR_ULTRASONIC_SIDEWAYS_POWER_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.vv_Constants.RANGESENSOR_OPTICAL_PROXIMITY_THRESHOLD;
import static org.firstinspires.ftc.teamcode.vv_Constants.RANGESENSOR_ULTRASONIC_PROXIMITY_THRESHOLD;
import static org.firstinspires.ftc.teamcode.vv_Constants.RIGHT_BEACON_BUTTON_SERVO;
import static org.firstinspires.ftc.teamcode.vv_Constants.ROBOT_TRACK_DISTANCE;
import static org.firstinspires.ftc.teamcode.vv_Constants.TURN_POWER;
import static org.firstinspires.ftc.teamcode.vv_Constants.TurnDirectionEnum;
import static org.firstinspires.ftc.teamcode.vv_Constants.GENERIC_TIMER;
import static org.firstinspires.ftc.teamcode.vv_Constants.DPAD_TIMER;

/**
 * Created by thomas on 9/25/2016.
 */

public class vv_Lib {

    protected vv_Robot robot;

    protected LineDetectCondition lineDetectStop = new LineDetectCondition();
    protected FalseCondition falseStop = new FalseCondition();
    protected EopdProximityCondition eopdProximityStop = new EopdProximityCondition();
    protected RangeSensorProximityOrColorVerifiedCondition rangeSensorProximityOrColorVerifiedStop =
            new RangeSensorProximityOrColorVerifiedCondition();
    protected RangeSensorOpticalProximityCondition rangeSensorOpticalProximityStop =
            new RangeSensorOpticalProximityCondition();
    protected RangeSensorUltraSonicProximityCondition rangeSensorUltraSonicProximityStop =
            new RangeSensorUltraSonicProximityCondition();
    protected RangeSensorUltraSonicCornerPositioningCondition rangeSensorUltraSonicCornerPositioningStop =
            new RangeSensorUltraSonicCornerPositioningCondition();
    protected colorPressVerifiedCondition colorPressVerifiedStop = new colorPressVerifiedCondition();




    float floorWhiteThreshold;

    public vv_Lib(vv_OpMode aOpMode)
            throws InterruptedException {

        CalibFileIO floorWhiteThresholdFileIO;
        floorWhiteThresholdFileIO = new CalibFileIO(aOpMode, "floorWhiteThreshold");
        try {
            floorWhiteThreshold = floorWhiteThresholdFileIO.getCalibrationValue(aOpMode);
            aOpMode.telemetryAddData("Floor White Threshold: ", String.valueOf(floorWhiteThreshold), "");
        } catch (IOException e) {
            aOpMode.telemetryAddData("Problem: ", e.getMessage(), "");
            floorWhiteThreshold = vv_Constants.FLOOR_WHITE_THRESHOLD;
            aOpMode.telemetryAddData("Floor White Threshold: ", String.valueOf(floorWhiteThreshold), "");
        } catch (NumberFormatException e) {
            aOpMode.telemetryAddData("Problem: ", e.getMessage(), "");
            floorWhiteThreshold = vv_Constants.FLOOR_WHITE_THRESHOLD;
            aOpMode.telemetryAddData("Floor White Threshold: ", String.valueOf(floorWhiteThreshold), "");
        }

        robot = new vv_Robot();
        robot.init(aOpMode, aOpMode.hardwareMap);

        //initialize stop conditions.

        float drivePowerFactor;
    }

    public vv_Lib()
            throws InterruptedException {

    //empty constructor used for some calls in TeleOp.
        //DO NOT USE unless you understand the use case.

    }


    /**
     * moveWheels method
     *
     * @param aOpMode   - object of vv_OpMode class
     * @param distance  - in centimeters
     * @param Power     - float
     * @param Direction - forward, backward, sideways left, or sideways right
     * @throws InterruptedException
     */
    public void moveWheels(vv_OpMode aOpMode, float distance, float Power,
                           DirectionEnum Direction, boolean isRampedPower)
            throws InterruptedException {
        if (Direction == Forward) {
            // moving the robot forward
            moveForwardToPosition(aOpMode, distance, Power, isRampedPower);
        } else if (Direction == Backward) {
            // moving the robot forward
            moveBackwardToPosition(aOpMode, distance, Power, isRampedPower);
        } else if (Direction == SidewaysLeft) {
            // moving the robot forward
            moveSidewaysLeftToPosition(aOpMode, distance, Power, isRampedPower);
        } else if (Direction == SidewaysRight) {
            // moving the robot forward
            moveSidewaysRightToPosition(aOpMode, distance, Power, isRampedPower);
        }
        // code for moving forward, backward, sideways
    }

    public void setupShot(vv_OpMode aOpMode) throws InterruptedException {
        robot.setupChooChoo(aOpMode);
    }

    public void dropBall(vv_OpMode aOpMode) throws InterruptedException {

        //close the rear launcher gate
        robot.closeRearLauncherGate();

        //open the launcher gate
        robot.openFrontLauncherGate();

        Thread.sleep(750);


        //wait for a ball to fall.
        robot.closeFrontLauncherGate();

        robot.openRearLauncherGate();

        //close the gate
    }


    public void shootBall(vv_OpMode aOpMode) throws InterruptedException {
        setupShot(aOpMode);
        robot.shootChooChoo(aOpMode);
    }

    /**
     * Using encoders, this method turns the Robot clockwise or counter clockwise based on angle given.
     * Calculates the turn distance by multiplying the angle by conversion factors to get to an encoder value
     *
     * @param aOpMode       an object of the vv_OpMode class
     * @param power         power in which to apply to each motor
     * @param angle         angle in which the robot will turn to based on the current position as 0 degree
     * @param TurnDirection Turns either Clockwise or Counterclockwise
     * @throws InterruptedException
     */
    public void turnUsingEncoders(vv_OpMode aOpMode, float power, float angle, TurnDirectionEnum TurnDirection)
            throws InterruptedException {

        //calculate the turn distance to be used in terms of encoder clicks.
        //for Andymark encoders.

        int turnDistance = (int) (2 * ((ROBOT_TRACK_DISTANCE) * angle
                * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION) /
                (MECCANUM_WHEEL_DIAMETER * 360));


        switch (TurnDirection) {
            case Clockwise:
                robot.runRobotToPosition(aOpMode, power, power, power, power, turnDistance, -turnDistance, turnDistance, -turnDistance, true);
                break;
            case Counterclockwise:
                robot.runRobotToPosition(aOpMode, power, power, power, power, -turnDistance, turnDistance, -turnDistance, turnDistance, true);
                break;
        }

        //wait just a bit for the commands to complete
        Thread.sleep(50);
    }


    public void turnUsingGyro(vv_OpMode aOpMode, float power, float angle, TurnDirectionEnum TurnDirection) {
        // do we need direction?
        // absolute vs. relative turns
    }


    public void showFloorColorSensorIntensityOnTelemetry(vv_OpMode aOpMode,
                                                         boolean updateTheDisplay)
            throws InterruptedException {


        aOpMode.telemetryAddData("Floor Color Sensor", "Light Intensity", ":" + robot.getFloorColorIntensity(aOpMode));
        if (updateTheDisplay) {
            aOpMode.telemetryUpdate();
        }
    }


    public void showBeaconLeftColorValuesOnTelemetry(vv_OpMode aOpMode,
                                                     boolean updateTheDisplay)
            throws InterruptedException {

        String color = "Unknown";

        if (robot.getBeaconLeftColor(aOpMode) == vv_Constants.BeaconColorEnum.RED) {
            color = "RED";
        }

        if (robot.getBeaconLeftColor(aOpMode) == vv_Constants.BeaconColorEnum.BLUE) {
            color = "BLUE";
        }
        if (robot.getBeaconLeftColor(aOpMode) == vv_Constants.BeaconColorEnum.UNKNOWN) {
            color = "Dont Know";
        }

        aOpMode.telemetryAddData("Beacon Left Red Val", "values:",
                "Red:" + robot.getBeaconLeftColorRedValue(aOpMode));
        aOpMode.telemetryAddData("Beacon Left Green Val", "values:",
                "Green:" + robot.getBeaconLeftColorGreenValue(aOpMode));
        aOpMode.telemetryAddData("Beacon Left Blue Val", "values:",
                "Blue:" + robot.getBeaconLeftColorBlueValue(aOpMode));

        aOpMode.telemetryAddData("Beacon LEFT  Color Choice", "Choice", color);

        if (updateTheDisplay) {
            aOpMode.telemetryUpdate();
        }
    }

    public void showBeaconRightColorValuesOnTelemetry(vv_OpMode aOpMode,
                                                      boolean updateTheDisplay)
            throws InterruptedException {

        String color = "Unknown";

        if (robot.getBeaconRightColor(aOpMode) == vv_Constants.BeaconColorEnum.RED) {
            color = "RED";
        }

        if (robot.getBeaconRightColor(aOpMode) == vv_Constants.BeaconColorEnum.BLUE) {
            color = "BLUE";
        }
        if (robot.getBeaconRightColor(aOpMode) == vv_Constants.BeaconColorEnum.UNKNOWN) {
            color = "Dont Know";
        }

        aOpMode.telemetryAddData("Beacon Right Red Val", "values:",
                "Red:" + robot.getBeaconRightColorRedValue(aOpMode));
        aOpMode.telemetryAddData("Beacon Right Green Val", "values:",
                "Green:" + robot.getBeaconRightColorGreenValue(aOpMode));
        aOpMode.telemetryAddData("Beacon Right Val", "values:",
                "Blue:" + robot.getBeaconRightColorBlueValue(aOpMode));

        aOpMode.telemetryAddData("Beacon RIGHT Color Choice", "Choice", color);

        if (updateTheDisplay) {
            aOpMode.telemetryUpdate();
        }
    }


    public void showMxpGyroSensorHeadingOnTelemetry(vv_OpMode aOpMode, boolean updateTheDisplay) {
        aOpMode.telemetryAddData("MXP Gyro Sensor", "Heading", ":" + robot.getMxpGyroSensorHeading(aOpMode));
        if (updateTheDisplay) {
            aOpMode.telemetryUpdate();
        }
    }

    public void showChooChooPositionOnTelemetry(vv_OpMode aOpMode, boolean updateTheDisplay) {

        aOpMode.telemetryAddData("Choo Choo", "Position", ":" + robot.getMotorPosition(aOpMode, ARM_MOTOR));
        if (updateTheDisplay) {
            aOpMode.telemetryUpdate();
        }
    }


    public void turnFloorColorSensorLedOn(vv_OpMode aOpMode) throws InterruptedException {
        robot.enableFloorColorSensorLed(aOpMode);
    }

    public void turnFloorColorSensorLedOff(vv_OpMode aOpMode) throws InterruptedException {
        robot.disableFloorColorSensorLed(aOpMode);
    }

    public void turnBeaconColorSensorLedOn(vv_OpMode aOpMode) throws InterruptedException {
        robot.enableBeaconLeftColorSensorLed(aOpMode);
    }

    public void turnBeaconColorSensorLedOff(vv_OpMode aOpMode) throws InterruptedException {
        robot.disableBeaconLeftColorSensorLed(aOpMode);
    }

    public vv_Constants.BeaconColorEnum getBeaconLeftColor(vv_OpMode aOpMode)
            throws InterruptedException {
        return robot.getBeaconLeftColor(aOpMode);
    }

    public vv_Constants.BeaconColorEnum getBeaconRightColor(vv_OpMode aOpMode)
            throws InterruptedException {
        return robot.getBeaconRightColor(aOpMode);
    }

    public double getFloorColorIntensity(vv_OpMode aOpMode) {
        return robot.getFloorColorIntensity(aOpMode);
    }


    //Moves robot forward with a distance supplied in centimeters and power between 0 and 1
    private void moveForwardToPosition(vv_OpMode aOpMode, float distance,
                                       float Power, boolean isRampedPower)
            throws InterruptedException {
        //we need to store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = (int) ((distance / (Math.PI * MECCANUM_WHEEL_DIAMETER)) * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //runs the robot to position
        robot.runRobotToPositionFB(aOpMode, targetPosition, Power, isRampedPower);
    }

    //Moves robot backward with a distance supplied in centimeters and power between 0 and 1
    private void moveBackwardToPosition(vv_OpMode aOpMode,
                                        float distance, float Power, boolean isRampedPower)
            throws InterruptedException {
        //we need to store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = -(int) ((distance / (Math.PI * MECCANUM_WHEEL_DIAMETER)) * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //runs the robot to position with negative power
        robot.runRobotToPositionFB(aOpMode, targetPosition, Power, isRampedPower);
    }

    private void moveSidewaysLeftToPosition(vv_OpMode aOpMode,
                                            float distance, float Power, boolean isRampedPower)
            throws InterruptedException {
        //we need to store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = (int) ((distance / (Math.PI * MECCANUM_WHEEL_DIAMETER)) * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //scale this up by 1.19 (due to sideways movement
        targetPosition = (int) Math.round(targetPosition * 1.19);

        //runs the robot to position with negative power
        robot.runRobotToPositionSideways(aOpMode, targetPosition, Power, isRampedPower);
    }

    private void moveSidewaysRightToPosition(vv_OpMode aOpMode,
                                             float distance, float Power, boolean isRampedPower)
            throws InterruptedException {
        //we need to store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = -(int) ((distance / (Math.PI * MECCANUM_WHEEL_DIAMETER)) * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //scale this up by 1.19 (due to sideways movement
        targetPosition = (int) Math.round(targetPosition * 1.19);

        //runs the robot to position with negative power
        robot.runRobotToPositionSideways(aOpMode, targetPosition, Power, isRampedPower);
    }




    public void runAllMotors(vv_OpMode aOpMode, float FLPower, float FRPower, float BLPower, float BRPower)
            throws InterruptedException {
        robot.runMotors(aOpMode, FLPower, FRPower, BLPower, BRPower);
    }

    public void stopAllMotors(vv_OpMode aOpMode) throws InterruptedException {
        robot.stopBaseMotors(aOpMode);
        //TODO: Stop additional motors that are not on the base of the Robot.
    }

    //Moves robot forward with a distance supplied in centimeters and power between 0 and 1
    public void moveForward(vv_OpMode aOpMode, float Power) throws InterruptedException {
        robot.runMotorsFB(aOpMode, Power);
    }

    //Moves robot backward with a distance supplied in centimeters and power between 0 and 1
    public void moveBackward(vv_OpMode aOpMode, float Power) throws InterruptedException {
        robot.runMotorsFB(aOpMode, -Power);
    }

    public double scalePowerForUltrasonicTravel(vv_OpMode aOpMode,
                                                double distanceToWall,//in inches
                                                double proximityDistance)
            throws InterruptedException { //in inches

        double distanceLeftToTravel = distanceToWall - proximityDistance; //could be negative!
        double scaledPower = MOTOR_ULTRASONIC_SIDEWAYS_POWER_LOWER_LIMIT + (0.02 * distanceLeftToTravel);
        if (scaledPower > MOTOR_ULTRASONIC_SIDEWAYS_POWER_UPPER_LIMIT) {
            scaledPower = MOTOR_ULTRASONIC_SIDEWAYS_POWER_UPPER_LIMIT;
        }

        return scaledPower;
    }

    public void turnAbsoluteMxpGyroDegrees(vv_OpMode aOpMode, float fieldReferenceDegrees) throws InterruptedException {
        //clockwise is represented by clockwise numbers.
        //counterclockwise by negative angle numbers in degrees.
        //the fieldReferenceDegrees parameters measures degrees off the initial reference frame when the robot is started and the gyro is
        //calibrated.
        // >> IMPORTANT: This depends on the zIntegratedHeading not being altered by relative turns !!!

        //first take the absolute degrees and modulus down to 0 and 359.

        float targetDegrees = fieldReferenceDegrees % 360;

        //compare to the current gyro zIntegrated heading and store the result.
        //the Integrated zValue returned is positive for clockwise turns
        //read the heading and store it.

        float startingHeading = robot.getMxpGyroSensorHeading(aOpMode);
        float turnDegrees = targetDegrees - startingHeading;

        //make the turn using encoders

        if (DEBUG) {
            aOpMode.telemetryAddData("targetDegrees", "Value",
                    ":" + targetDegrees);
            aOpMode.telemetryAddData("Starting Z", "Value",
                    ":" + startingHeading);
            aOpMode.telemetryAddData("Turn Degrees", "Value",
                    ":" + turnDegrees);

            aOpMode.telemetryUpdate();
        }

        //optimize the turn, so that direction of turn results in smallest turn needed.

        if (Math.abs(turnDegrees) > 180) {
            turnDegrees = Math.signum(turnDegrees) * -1 * (360 - Math.abs(turnDegrees));
        }

        turnUsingEncoders(aOpMode, TURN_POWER, Math.abs(turnDegrees),
                turnDegrees > 0 ? TurnDirectionEnum.Clockwise :
                        TurnDirectionEnum.Counterclockwise);

        float finalDegrees = robot.getMxpGyroSensorHeading(aOpMode);
        Thread.sleep(25); //cooling off after gyro read to prevent error in next run.

        if (DEBUG) {
            aOpMode.telemetryAddData("New Bearing Degrees", "Value:",
                    ":" + finalDegrees);
            aOpMode.telemetryAddData("Turn Error Degrees", "Value:",
                    ":" + (targetDegrees - finalDegrees));
            aOpMode.telemetryUpdate();
        }

    }



    public void turnPidMxpAbsoluteDegrees(vv_OpMode aOpMode, float turndegrees, float toleranceDegrees)
            throws InterruptedException {

        robot.turnPidMxpAbsoluteDegrees(aOpMode, turndegrees, toleranceDegrees);

    }


    /**
     * Toggles the power of the Ball Collector Motor to either off or to the power required to
     * outtake depending on the current state of the motor
     *
     * @param anOp an object of vv_OpMode
     */
    public void toggleOuttake(vv_OpMode anOp) throws InterruptedException {
        // if the current ball collector state is Outtake or Intake, turn the motor off
        // else set the ball collector motor power to the outtake power
        if (robot.getIntakeState() == vv_Constants.IntakeStateEnum.Out ||
                robot.getIntakeState() == vv_Constants.IntakeStateEnum.In) {
            robot.setPower(anOp, INTAKE_MOTOR, 0.0f);
            robot.setIntakeState(vv_Constants.IntakeStateEnum.Off);
        } else {
            robot.setPower(anOp, INTAKE_MOTOR, vv_Constants.INTAKE_POWER); //TODO: Check negate
            robot.setIntakeState(vv_Constants.IntakeStateEnum.Out);
        }
    }

    /**
     * Toggles the power of the Ball Collector Motor to either off or to the power required to
     * intake depending on the current state of the motor
     *
     * @param anOp an object of vv_OpMode
     */
    public void toggleIntake(vv_OpMode anOp)
            throws InterruptedException {
        // if the current ball collector state is Outtake or Intake, turn the motor off
        // else set the ball collector motor power to the intake power
        if (robot.getIntakeState() == vv_Constants.IntakeStateEnum.In ||
                robot.getIntakeState() == vv_Constants.IntakeStateEnum.Out) {
            //first close the rear intake servo
            robot.closeRearLauncherGate();
            robot.setPower(anOp, INTAKE_MOTOR, 0.0f);
            robot.setIntakeState(vv_Constants.IntakeStateEnum.Off);
        } else {
            //first raise the rear intake servo
            robot.openRearLauncherGate();
            robot.setPower(anOp, INTAKE_MOTOR, -vv_Constants.INTAKE_POWER); //TODO: Check negate
            robot.setIntakeState(vv_Constants.IntakeStateEnum.In);
        }
    }

    public void universalMoveRobot(vv_OpMode aOpMode, double polarAngle,
                                   double polarVelocity, double rotationalVelocity,
                                   long duration, vv_OpMode.StopCondition condition,
                                   boolean isPulsed, long pulseWidthDuration, long pulseRestDuration)
            throws InterruptedException {


        robot.universalMoveRobot(aOpMode, polarVelocity * Math.sin(Math.toRadians(polarAngle)),
                polarVelocity * Math.cos(Math.toRadians(polarAngle)), rotationalVelocity, duration, condition, isPulsed, pulseWidthDuration, pulseRestDuration);
    }


    public void universalMoveRobotForFieldOrientedTeleOp(vv_OpMode aOpMode, double polarMagnitude,
                                                         double polarAngle)
            throws InterruptedException {

        robot.universalMoveRobotForTeleOp(aOpMode,
                polarMagnitude * Math.sin(Math.toRadians(polarAngle)), //trig in radians
                polarMagnitude * Math.cos(Math.toRadians(polarAngle)));
    }



    public void decreaseLauncherPower(vv_OpMode aOpMode) throws InterruptedException,
            vv_Robot.MotorStalledException {
        int launcherPowerPosition = robot.getLauncherPowerPosition(aOpMode);

        launcherPowerPosition -= LAUNCH_POWER_INCREMENT;
        robot.setLauncherPowerPosition(aOpMode, launcherPowerPosition);
    }

    public void increaseLauncherPower(vv_OpMode aOpMode) throws InterruptedException,
            vv_Robot.MotorStalledException {
        int launcherPowerPosition = robot.getLauncherPowerPosition(aOpMode);


        launcherPowerPosition += LAUNCH_POWER_INCREMENT;


        robot.setLauncherPowerPosition(aOpMode, launcherPowerPosition);
    }

    public void setLauncherPowerPosition(vv_OpMode aOpMode, int position)
            throws InterruptedException, vv_Robot.MotorStalledException {
        robot.setLauncherPowerPosition(aOpMode, position);
    }

    public double getFrontLauncherGatePosition(vv_OpMode aOpMode) {
        return robot.getFrontLauncherGateServoPosition(aOpMode);
    }

    public void setFrontLauncherGatePosition(vv_OpMode aOpMode, double position) {
        robot.setFrontLauncherGateServoPosition(aOpMode, position);
    }


    public int getLauncherPowerPosition(vv_OpMode aOpMode) {
        return robot.getLauncherPowerPosition(aOpMode);
    }


    public double getUltrasonicDistance(vv_OpMode aOpMode)
            throws InterruptedException {
        return robot.getUltrasonicDistance(aOpMode);
    }

    public double getOpticalDistance(vv_OpMode aOpMode)
            throws InterruptedException {
        return robot.getOpticalDistance(aOpMode);
    }


    public double getBeaconServoPosition(vv_OpMode aOpMode, int servoName) {
        return robot.getBeaconServoPosition(aOpMode, servoName);
    }

    public void setBeaconServoPosition(vv_OpMode aOpMode, int servoName, double position)
            throws InterruptedException {
        robot.setBeaconServoPosition(aOpMode, servoName, position);
    }

    public void pressLeftBeaconButton(vv_OpMode aOpMode) throws InterruptedException {
        robot.setBeaconServoPosition(aOpMode, LEFT_BEACON_BUTTON_SERVO, BEACON_SERVO_LEFT_PRESSED);
        Thread.sleep(200);
        robot.setBeaconServoPosition(aOpMode, LEFT_BEACON_BUTTON_SERVO, BEACON_SERVO_LEFT_REST);
    }

    public void pressRightBeaconButton(vv_OpMode aOpMode) throws InterruptedException {
        robot.setBeaconServoPosition(aOpMode, RIGHT_BEACON_BUTTON_SERVO, BEACON_SERVO_RIGHT_PRESSED);
        Thread.sleep(200);
        robot.setBeaconServoPosition(aOpMode, RIGHT_BEACON_BUTTON_SERVO, BEACON_SERVO_RIGHT_REST);
    }

    public void extendLeftBeaconButtonPress(vv_OpMode aOpMode) throws InterruptedException {
        robot.setBeaconServoPosition(aOpMode, LEFT_BEACON_BUTTON_SERVO, BEACON_SERVO_LEFT_PRESSED);
    }

    public void extendRightBeaconButtonPress(vv_OpMode aOpMode) throws InterruptedException {
        robot.setBeaconServoPosition(aOpMode, RIGHT_BEACON_BUTTON_SERVO, BEACON_SERVO_RIGHT_PRESSED);
    }

    public void closeLeftBeaconButtonPress(vv_OpMode aOpMode) throws InterruptedException {
        robot.setBeaconServoPosition(aOpMode, LEFT_BEACON_BUTTON_SERVO, BEACON_SERVO_LEFT_REST);
    }

    public void closeRightBeaconButton(vv_OpMode aOpMode) throws InterruptedException {
        robot.setBeaconServoPosition(aOpMode, RIGHT_BEACON_BUTTON_SERVO, BEACON_SERVO_RIGHT_REST);
    }

    public void raiseBallFlagServo(vv_OpMode aOpMode) throws InterruptedException {
        robot.setBallFlagServoPosition(aOpMode, BALL_FLAG_SERVO_RAISED);
    }

    public void lowerBallFlagServo(vv_OpMode aOpMode) throws InterruptedException {
        robot.setBallFlagServoPosition(aOpMode, BALL_FLAG_SERVO_LOWERED);
    }

    public void showRangeSensorDistanceOnTelemetry(vv_OpMode aOpMode) {
        aOpMode.telemetryAddData("MR Range Ultrasonic Distance",
                " in inches:", " " + robot.getUltrasonicDistance(aOpMode));

        aOpMode.telemetryAddData("MR Range Optical Distance",
                " in inches:", " " + robot.getOpticalDistance(aOpMode));
        aOpMode.telemetryUpdate();
    }

    public void showEopdValueOnTelemetry(vv_OpMode aOpMode) {
        aOpMode.telemetryAddData("EOPD raw Value:", "Value", ":" +
                robot.getEopdRawValue(aOpMode));
        aOpMode.telemetryUpdate();
    }

    public void showGamepad1PolarCoordinates(vv_OpMode aOpMode) {
        aOpMode.telemetryAddData("Rot Left Stick:", "Angle:", ":" +
                robot.getGamePad1LeftJoystickPolarAngle(aOpMode) + "degrees ");
        aOpMode.telemetryAddData("Rot Left Stick:", "Magnitude:", ":" +
                robot.getGamePad1LeftJoystickPolarMagnitude(aOpMode));

        aOpMode.telemetryAddData("Rot Right Stick:", "Angle:", ":" +
                robot.getGamePad1RightJoystickPolarAngle(aOpMode) + "degrees ");
        aOpMode.telemetryAddData("Rot Right Stick:", "Magnitude:", ":" +
                robot.getGamePad1RightJoystickPolarMagnitude(aOpMode));

        aOpMode.telemetryUpdate();
    }

    public double getEopdRawValue(vv_OpMode aOpMode) {

        return robot.getEopdRawValue(aOpMode);
    }


    public void detectColorAndPressBeacon(vv_OpMode aOpMode,
                                          vv_Constants.BeaconColorEnum teamColor) throws
            InterruptedException {


        if (teamColor == vv_Constants.BeaconColorEnum.BLUE) {
            //team blue
            if (getBeaconLeftColor(aOpMode) == vv_Constants.BeaconColorEnum.BLUE) {
                //found blue
                //press left beacon button
                extendLeftBeaconButtonPress(aOpMode);
            } else if (getBeaconLeftColor(aOpMode) == vv_Constants.BeaconColorEnum.RED) {
                //found red
                //press right button
                extendRightBeaconButtonPress(aOpMode);
            }
        }
        if (teamColor == vv_Constants.BeaconColorEnum.RED) {
            //team red
            if (getBeaconLeftColor(aOpMode) == vv_Constants.BeaconColorEnum.RED) {
                //found red
                //press left beacon button
                extendLeftBeaconButtonPress(aOpMode);
            } else if (getBeaconLeftColor(aOpMode) == vv_Constants.BeaconColorEnum.BLUE) {
                //found blue
                //press right button
                extendRightBeaconButtonPress(aOpMode);
            }
        }


        //move forward to press beacon button.
        //lets keep pulsing forward until the color changes or time runs out or proximity limits
        //are reached

        universalMoveRobot(aOpMode, 90, 0.3, 0.0, 2000, new
                RangeSensorProximityOrColorVerifiedCondition(), true, 200, 10);


        //now retract both beacon presses
        closeLeftBeaconButtonPress(aOpMode);
        closeRightBeaconButton(aOpMode);
    }

    public void ScoreBeaconFromTheRight(vv_OpMode aOpMode) throws InterruptedException {

        //assume the robot is at right angles and facing the beacon but to the left of the white line
        //when this method is called.


        //read distance from ultrasonic sensor, noise filtered, with 7 readings in a set.
        double distanceToBeaconWall = getUltrasonicDistance(aOpMode); //in inches
        //now try moving that distance, adjusting for inset of ultrasonic sensor
        //move toward the beacons but stop short (approx 1.5 inches short).
        moveWheels(aOpMode, (float) (distanceToBeaconWall - 3.25), 0.7f, SidewaysRight, true);

        //now detect the line but at right angles
        //for first beacon


        universalMoveRobot(aOpMode, 0, 0.25, 0.0, 3000, lineDetectStop, false, 0, 0);
        //now detect the line but at right angles

        Thread.sleep(25);

        moveWheels(aOpMode, 5.5f, 0.4f, Backward, false); // adjust face position to match beacons

        Thread.sleep(25);

        turnAbsoluteMxpGyroDegrees(aOpMode, 90); //with trim, readjust to prep for color read

        detectColorAndPressBeacon(aOpMode, vv_Constants.BeaconColorEnum.BLUE);

        Thread.sleep(25);
    }

    public void ScoreBeaconFromTheLeft(vv_OpMode aOpMode) throws InterruptedException {

        //assume the robot is at right angles and facing the beacon but to the right of the white line
        //when this method is called.


        //read distance from ultrasonic sensor, noise filtered, with 7 readings in a set.
        double distanceToBeaconWall = getUltrasonicDistance(aOpMode); //in inches
        //now try moving that distance, adjusting for inset of ultrasonic sensor
        //move toward the beacons but stop short (approx 1.5 inches short).
        moveWheels(aOpMode, (float) (distanceToBeaconWall - 3.5), 0.7f, SidewaysRight, true);

        Thread.sleep(25);

        //now detect the line but at right angles
        //for first beacon


        universalMoveRobot(aOpMode, 180, 0.20, 0.0, 3000, lineDetectStop, false, 0, 0);
        //now detect the line but at right angles

        Thread.sleep(25);

        turnAbsoluteMxpGyroDegrees(aOpMode, -90); //with trim, readjust to prep for color read

        Thread.sleep(25);

        //moveWheels(aOpMode, 2.5f, 0.20f, Backward, false); // adjust face position to match beacons

        Thread.sleep(25);

        turnAbsoluteMxpGyroDegrees(aOpMode, -90); //with trim, readjust to prep for color read

        detectColorAndPressBeacon(aOpMode, vv_Constants.BeaconColorEnum.RED);


    }

    public void blueBeaconAutonomousCommonAction(vv_OpMode aOpMode) throws InterruptedException {
        //rotate to face beacon
        Thread.sleep(25);
        turnAbsoluteMxpGyroDegrees(aOpMode, 90); //with trim
        Thread.sleep(25);
        //detect the line and score beacon.

        ScoreBeaconFromTheRight(aOpMode);

        //now move to second beacon

        //pull back 4 inches to create clearance
        moveWheels(aOpMode, 4, 0.8f, SidewaysLeft, true);
        Thread.sleep(25);
        //orient to 90 degrees to field

        turnAbsoluteMxpGyroDegrees(aOpMode, 90); //with trim
        Thread.sleep(25);
        //lets move over the first beacon line, to prevent stopping at wrong line.

        moveWheels(aOpMode, 40f, 0.99f, Forward, true);
        Thread.sleep(25);
        turnAbsoluteMxpGyroDegrees(aOpMode, 90); //with trim
        Thread.sleep(25);
        ScoreBeaconFromTheRight(aOpMode);
        Thread.sleep(25);
        //knock off the cap ball.
        moveWheels(aOpMode, 10, .8f, SidewaysLeft, true);
        Thread.sleep(25);
        turnAbsoluteMxpGyroDegrees(aOpMode, 140);
        Thread.sleep(25);
        moveWheels(aOpMode, 57, 0.99f, Backward, false);

    }

    public void redBeaconAutonomousCommonAction(vv_OpMode aOpMode) throws InterruptedException {
        //rotate to face beacon
        Thread.sleep(50);
        turnAbsoluteMxpGyroDegrees(aOpMode, -90); //with trim

        //detect the line and score beacon.

        ScoreBeaconFromTheLeft(aOpMode);

        //now to work on second beacon.


        //now move to second beacon

        //pull back 6 inches to create clearance
        moveWheels(aOpMode, 6, 0.8f, SidewaysLeft, true);

        //orient to 90 degrees to field

        Thread.sleep(25);

        turnAbsoluteMxpGyroDegrees(aOpMode, -90); //with trim
        Thread.sleep(25);

        //lets move over the first beacon line, to prevent stopping at wrong line.

        moveWheels(aOpMode, 35.0f, 0.99f, Backward, true);

        turnAbsoluteMxpGyroDegrees(aOpMode, -90); //with trim
        Thread.sleep(25);

        ScoreBeaconFromTheLeft(aOpMode);

        moveWheels(aOpMode, 10, .8f, SidewaysLeft, true);
        Thread.sleep(25);
        turnAbsoluteMxpGyroDegrees(aOpMode, -135);
        Thread.sleep(25);
        moveWheels(aOpMode, 55.0f, .99f, Forward, true);


    }

    public void shootBallsAutonomousCommonAction(vv_OpMode aOpMode)
            throws InterruptedException {


        //lets check for the choo choo arm being in the right position
        //if not move into the right position.

        robot.setChooChooToLimit(aOpMode);
        // Shoot the first ball
        shootBall(aOpMode);
        setupShot(aOpMode);

        //drop ball
        dropBall(aOpMode);

        //Shoot the second ball.
        shootBall(aOpMode);

        //rotate back for reference before proceeding
        turnAbsoluteMxpGyroDegrees(aOpMode, 0f);
    }

    public void blueAlternateAutonomous(vv_OpMode aOpMode) throws InterruptedException {

        //delay
        Thread.sleep(10000);

        moveWheels(aOpMode, 20, .99f, DirectionEnum.SidewaysRight, false);
        turnAbsoluteMxpGyroDegrees(aOpMode, -15);

        // Shoot the first ball
        shootBall(aOpMode);
        setupShot(aOpMode);

        //drop ball
        dropBall(aOpMode);

        //Shoot the second ball.
        shootBall(aOpMode);
        shootBall(aOpMode);

        setupShot(aOpMode);

        turnAbsoluteMxpGyroDegrees(aOpMode, -90);
        moveWheels(aOpMode, 43, .99f, DirectionEnum.Backward, true);
    }

    public void redAlternateAutonomous (vv_OpMode aOpMode) throws InterruptedException {

        //delay
        Thread.sleep(10000);
        moveWheels(aOpMode, 14, .99f, DirectionEnum.SidewaysRight, false);
        turnAbsoluteMxpGyroDegrees(aOpMode, -15);

        // Shoot the first ball
        shootBall(aOpMode);
        setupShot(aOpMode);

        //drop ball
        dropBall(aOpMode);

        //Shoot the second ball.
        shootBall(aOpMode);
        shootBall(aOpMode);

        setupShot(aOpMode);

        turnAbsoluteMxpGyroDegrees(aOpMode, -86);
        moveWheels(aOpMode, 53, .99f, DirectionEnum.Backward, true);
    }


    public void shootBallAndSpinIntake(vv_OpMode aOpMode) throws InterruptedException {
        setupShot(aOpMode);
        dropBall(aOpMode);
        shootBall(aOpMode);
        //spin intake
        robot.openRearLauncherGate();
        aOpMode.reset_timer_array(GENERIC_TIMER);

        robot.setPower(aOpMode, INTAKE_MOTOR,
                -vv_Constants.INTAKE_POWER);
        setupShot(aOpMode);
        //stop intake
        while (aOpMode.time_elapsed_array(GENERIC_TIMER) < 2750 && getEopdRawValue(aOpMode) < EOPD_PROXIMITY_THRESHOLD) {
            //spin till we are past time limit or ball is detected in launch tube.
        }

        robot.closeRearLauncherGate();

        robot.setPower(aOpMode, INTAKE_MOTOR, 0);


    }


    public boolean isRobotAtAngle(vv_OpMode aOpMode, int angle) {
        if(Math.abs(robot.getMxpGyroSensorHeading(aOpMode) - angle) < vv_Constants.ROBOT_ANGLE_THRESHOLD) {
            return true;
        } else {
            return false;
        }
    }

    //conditions that can stop the robot.


    public class LineDetectCondition implements vv_OpMode.StopCondition {

        public boolean stopCondition(vv_OpMode aOpMode) throws InterruptedException {
            return ((getFloorColorIntensity(aOpMode) >= (floorWhiteThreshold-FLOOR_WHITE_MARGIN)));
        }
    }

    public class FalseCondition implements vv_OpMode.StopCondition {
        //can be used as an empty condition, so the robot keeps running in universal movement
        public boolean stopCondition(vv_OpMode aOpMode) throws InterruptedException {
            return (false);
        }
    }


    public class EopdProximityCondition implements vv_OpMode.StopCondition {
        public boolean stopCondition(vv_OpMode aOpMode) throws InterruptedException {
            return (getEopdRawValue(aOpMode) > EOPD_PROXIMITY_THRESHOLD);
        }
    }

    public class RangeSensorProximityOrColorVerifiedCondition implements vv_OpMode.StopCondition {
        public boolean stopCondition(vv_OpMode aOpMode) throws InterruptedException {
            return (((getOpticalDistance(aOpMode) < RANGESENSOR_OPTICAL_PROXIMITY_THRESHOLD)
                    && getOpticalDistance(aOpMode) > 0) ||
                    (getUltrasonicDistance(aOpMode)
                            < RANGESENSOR_ULTRASONIC_PROXIMITY_THRESHOLD) ||
                    (getBeaconLeftColor(aOpMode) == getBeaconRightColor(aOpMode)));

        }
    }

    public class RangeSensorOpticalProximityCondition implements vv_OpMode.StopCondition {
        public boolean stopCondition(vv_OpMode aOpMode) throws InterruptedException {
            return (((getOpticalDistance(aOpMode) < RANGESENSOR_OPTICAL_PROXIMITY_THRESHOLD)
                    && getOpticalDistance(aOpMode) > 0));

        }
    }

    public class RangeSensorUltraSonicProximityCondition implements vv_OpMode.StopCondition {
        public boolean stopCondition(vv_OpMode aOpMode) throws InterruptedException {
            return (getUltrasonicDistance(aOpMode)
                    < RANGESENSOR_ULTRASONIC_PROXIMITY_THRESHOLD);

        }
    }

    public class RangeSensorUltraSonicCornerPositioningCondition implements vv_OpMode.StopCondition {
        public boolean stopCondition(vv_OpMode aOpMode) throws InterruptedException {

            //TODO: This code is not functioning because we are not facing the right wall to do this.
            return (getUltrasonicDistance(aOpMode)
                    < 2 * RANGESENSOR_ULTRASONIC_PROXIMITY_THRESHOLD);
        }
    }


    public class colorPressVerifiedCondition implements vv_OpMode.StopCondition {
        public boolean stopCondition(vv_OpMode aOpMode) throws InterruptedException {
            //button is pressed because both colors match , not a strong test but a good starting point for
            //teleop.
            return (getBeaconLeftColor(aOpMode) == getBeaconRightColor(aOpMode));

        }
    }


}
