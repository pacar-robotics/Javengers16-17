package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.vv_Constants.ANALOG_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.vv_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.vv_Constants.ARM_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.Backward;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.Forward;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysLeft;
import static org.firstinspires.ftc.teamcode.vv_Constants.DirectionEnum.SidewaysRight;
import static org.firstinspires.ftc.teamcode.vv_Constants.FLOOR_WHITE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.GYRO_OFFSET;
import static org.firstinspires.ftc.teamcode.vv_Constants.INTAKE_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.LAUNCH_POWER_INCREMENT;
import static org.firstinspires.ftc.teamcode.vv_Constants.LAUNCH_POWER_POSITION_MAX;
import static org.firstinspires.ftc.teamcode.vv_Constants.LAUNCH_POWER_POSITION_MIN;
import static org.firstinspires.ftc.teamcode.vv_Constants.MAX_MOTOR_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.vv_Constants.MAX_ROBOT_TURN_MOTOR_VELOCITY;
import static org.firstinspires.ftc.teamcode.vv_Constants.MECCANUM_WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.vv_Constants.MIN_ROBOT_TURN_MOTOR_VELOCITY;
import static org.firstinspires.ftc.teamcode.vv_Constants.ROBOT_TRACK_DISTANCE;
import static org.firstinspires.ftc.teamcode.vv_Constants.TOUCH_SENSE_POWER;
import static org.firstinspires.ftc.teamcode.vv_Constants.TURN_POWER;
import static org.firstinspires.ftc.teamcode.vv_Constants.TurnDirectionEnum;


/**
 * Created by thomas on 9/25/2016.
 */

public class vv_Lib {
    private vv_Robot robot;


    public vv_Lib(vv_OpMode aOpMode)
            throws InterruptedException {
        robot = new vv_Robot();
        robot.init(aOpMode, aOpMode.hardwareMap);
        setupShot(aOpMode);
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
            throws InterruptedException {
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

    public void setupShot(vv_OpMode aOpMode) throws InterruptedException
    {


        robot.setMotorMode(aOpMode, ARM_MOTOR, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        aOpMode.reset_timer();
        while (!robot.isArmAtLimit(aOpMode) && aOpMode.time_elapsed() < MAX_MOTOR_LOOP_TIME) {

            robot.setPower(aOpMode, ARM_MOTOR, 1.0f);
            aOpMode.idle();
        }
        robot.setPower(aOpMode, ARM_MOTOR, 0.0f);



    }

    public void dropBall(vv_OpMode aOpMode) throws InterruptedException {

        //open the launcher gate
        robot.openLauncherGate();
        Thread.sleep(550);
        //wait for a ball to fall.
        robot.closeLauncherGate();
        //close the gate
    }



    public void shootBall(vv_OpMode aOpMode) throws InterruptedException
    {
        robot.setMotorMode(aOpMode, ARM_MOTOR, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.setPower(aOpMode, ARM_MOTOR, 0.8f);

        Thread.sleep(500);

        robot.setPower(aOpMode, ARM_MOTOR, 0.0f);
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
            throws InterruptedException {

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

    public void turnBeaconArm(vv_OpMode aOpMode, vv_Constants.BeaconServoStateEnum BeaconServoStateEnumValue)
            throws InterruptedException {
        robot.turnBeaconArm(aOpMode, BeaconServoStateEnumValue);
    }


    public void turnUsingGyro(vv_OpMode aOpMode, float power, float angle, TurnDirectionEnum TurnDirection) {
        // do we need direction?
        // absolute vs. relative turns
    }


    public boolean senseTouch(vv_OpMode aOpMode) throws InterruptedException {
        return robot.getButtonTouchValue(aOpMode);
    }

    public void moveTillTouch(vv_OpMode aOpMode) throws InterruptedException {
        aOpMode.reset_timer();
        while (!senseTouch(aOpMode) && aOpMode.time_elapsed() < MAX_MOTOR_LOOP_TIME) {
            robot.runMotorsSidewaysRight(aOpMode, TOUCH_SENSE_POWER);
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
     * @throws InterruptedException
     */
    public void moveTillWhiteLineDetect(vv_OpMode aOpMode, float Power) throws InterruptedException {
        aOpMode.reset_timer();
        while ((robot.getFloorLightIntensity(aOpMode) < FLOOR_WHITE_THRESHOLD) &&
                aOpMode.time_elapsed() < MAX_MOTOR_LOOP_TIME) {
            moveSidewaysRight(aOpMode, Power);
            aOpMode.idle();
        }
        //stop motors
        robot.stopBaseMotors(aOpMode);
    }


    public void showFloorLightSensorIntensityOnTelemetry(vv_OpMode aOpMode,
                                                         boolean updateTheDisplay)
            throws InterruptedException {


        aOpMode.telemetryAddData("Floor Sensor", "Light Intensity", ":" + robot.getFloorLightIntensity(aOpMode));
        if (updateTheDisplay) {
            aOpMode.telemetryUpdate();
        }
    }

    public void showBeaconColorValuesOnTelemetry(vv_OpMode aOpMode,
                                                 boolean updateTheDisplay)
            throws InterruptedException {

        String color = "Unknown";

        if (robot.getBeaconColor(aOpMode) == vv_Constants.BeaconColorEnum.RED) {
            color = "RED";
        }

        if (robot.getBeaconColor(aOpMode) == vv_Constants.BeaconColorEnum.BLUE) {
            color = "BLUE";
        }
        if (robot.getBeaconColor(aOpMode) == vv_Constants.BeaconColorEnum.UNKNOWN) {
            color = "Dont Know";
        }

        aOpMode.telemetryAddData("Beacon red Val", "values:",
                "Red:" + robot.getBeaconColorRedValue(aOpMode));
        aOpMode.telemetryAddData("Beacon red Val", "values:",
                "Green:" + robot.getBeaconColorGreenValue(aOpMode));
        aOpMode.telemetryAddData("Beacon red Val", "values:",
                "Blue:" + robot.getBeaconColorBlueValue(aOpMode));

        aOpMode.telemetryAddData("Beacon Color Choice", "Choice", color);

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


    public void turnFloorLightSensorLedOn(vv_OpMode aOpMode) throws InterruptedException {
        robot.enableFloorLightSensorLed(aOpMode);
    }

    public void turnFloorColorSensorLedOff(vv_OpMode aOpMode)throws InterruptedException{
        robot.disableFloorLightSensorLed(aOpMode);
    }

    public void turnBeaconColorSensorLedOn(vv_OpMode aOpMode) throws InterruptedException {
        robot.enableBeaconColorSensorLed(aOpMode);
    }

    public void turnBeaconColorSensorLedOff(vv_OpMode aOpMode) throws InterruptedException {
        robot.disableBeaconColorSensorLed(aOpMode);
    }

    public vv_Constants.BeaconColorEnum getBeaconColor(vv_OpMode aOpMode)
            throws InterruptedException {
        return robot.getBeaconColor(aOpMode);
    }

    //Moves robot forward with a distance supplied in centimeters and power between 0 and 1
    private void moveForwardToPosition(vv_OpMode aOpMode, float distance, float Power)
            throws InterruptedException {
        //we need to store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = (int) ((distance / (Math.PI * MECCANUM_WHEEL_DIAMETER)) * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //runs the robot to position
        robot.runRobotToPositionFB(aOpMode, targetPosition, Power);
    }

    //Moves robot backward with a distance supplied in centimeters and power between 0 and 1
    private void moveBackwardToPosition(vv_OpMode aOpMode, float distance, float Power)
            throws InterruptedException {
        //we need to store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = -(int) ((distance / (Math.PI * MECCANUM_WHEEL_DIAMETER)) * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //runs the robot to position with negative power
        robot.runRobotToPositionFB(aOpMode, targetPosition, Power);
    }

    private void moveSidewaysLeftToPosition(vv_OpMode aOpMode, float distance, float Power)
            throws InterruptedException {
        //we need to store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = (int) ((distance / (Math.PI * MECCANUM_WHEEL_DIAMETER)) * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //runs the robot to position with negative power
        robot.runRobotToPositionSideways(aOpMode, targetPosition, Power);
    }

    private void moveSidewaysRightToPosition(vv_OpMode aOpMode, float distance, float Power)
            throws InterruptedException {
        //we need to store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = -(int) ((distance / (Math.PI * MECCANUM_WHEEL_DIAMETER)) * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //runs the robot to position with negative power
        robot.runRobotToPositionSideways(aOpMode, targetPosition, Power);
    }


    //DO NOT USE THIS METHOD
    //IT IS NOT COMPLETED
    public void moveAtAngle(vv_OpMode aOpMode, double distance, float Power, float Angle)
            throws InterruptedException {
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

    public void moveSidewaysLeft(vv_OpMode aOpMode, float Power) throws InterruptedException {
        robot.runMotorsSidewaysLeft(aOpMode, Power);
    }

    public void moveSidewaysRight(vv_OpMode aOpMode, float Power) throws InterruptedException {
        robot.runMotorsSidewaysRight(aOpMode, Power);
    }

    public void turnGyroDegrees(vv_OpMode aOpMode, int turnDegrees) throws InterruptedException {

        //this code has some issues due to gyro read lag of approximately 250ms as reported.
        //This will cause the turns to be approximate and will generally overshoot.
        //only use this code if other methods dont work.
        //we will be trying this with another more advanced gyro sensor.


        float FRONT_LEFT_MOTORPower = 0;
        float FRONT_RIGHT_MOTORPower = 0;
        float BACK_LEFT_MOTORPower = 0;
        float BACK_RIGHT_MOTORPower = 0;


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

            FRONT_LEFT_MOTORPower = -1 * turnPower;
            FRONT_RIGHT_MOTORPower = turnPower;
            BACK_LEFT_MOTORPower = -1 * turnPower;
            BACK_RIGHT_MOTORPower = turnPower;

            //re-scan zValue
            zValue = robot.getBaseGyroSensorIntegratedZValue(aOpMode);
            if (Math.abs(zValue) >= Math.abs(turnDegrees)) {
                //break out of loop.
                break;
            }

            robot.runMotors(aOpMode, FRONT_LEFT_MOTORPower, FRONT_RIGHT_MOTORPower,
                    BACK_LEFT_MOTORPower, BACK_RIGHT_MOTORPower);

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

    public void turnAbsoluteGyroDegrees(vv_OpMode aOpMode, float fieldReferenceDegrees) throws InterruptedException {
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

    public void drive1RobotWithPowerFactor(vv_OpMode aOpMode, float powerFactor)
            throws InterruptedException, vv_Robot.MotorNameNotKnownException {

        //left stick is for forward/backward motion.
        //right stick for turns.

        //read forward/backward movements from the left stick y axis and apply it to the motors.

        float frontLeftMotorPower = 0;
        float frontRightMotorPower = 0;
        float backLeftMotorPower = 0;
        float backRightMotorPower = 0;

        if (Math.abs(aOpMode.gamepad1.right_stick_x) < ANALOG_STICK_THRESHOLD) {
            //we think the driver is not using the right stick to turn
            if (Math.abs(aOpMode.gamepad1.left_stick_y) > ANALOG_STICK_THRESHOLD) {
                //lets drive the motor forward or back based on the stick values.

                frontLeftMotorPower = aOpMode.gamepad1.left_stick_y * powerFactor;
                frontRightMotorPower = aOpMode.gamepad1.left_stick_y * powerFactor;
                backLeftMotorPower = aOpMode.gamepad1.left_stick_y * powerFactor;
                backRightMotorPower = aOpMode.gamepad1.left_stick_y * powerFactor;
                //apply the power
                robot.runMotors(aOpMode, frontLeftMotorPower, frontRightMotorPower,
                        backLeftMotorPower, backRightMotorPower);

            }
        } else {
            frontLeftMotorPower = Math.abs(aOpMode.gamepad1.right_stick_x * powerFactor);
            frontRightMotorPower = Math.abs(aOpMode.gamepad1.right_stick_x * powerFactor);
            backLeftMotorPower = Math.abs(aOpMode.gamepad1.right_stick_x * powerFactor);
            backRightMotorPower = Math.abs(aOpMode.gamepad1.right_stick_x * powerFactor);

            if (aOpMode.gamepad1.right_stick_x > 0) {
                //clockwise turn
                frontRightMotorPower = -frontRightMotorPower;
                backRightMotorPower = -backRightMotorPower;
            } else {
                //counter clockwise turn
                frontLeftMotorPower = -frontLeftMotorPower;
                backLeftMotorPower = -backLeftMotorPower;
            }
        }

        //when power is zero as initialized the motors will stop. This should be true if the
        //joysticks are released.
        //otherwise this will run the motors in desired direction.

        robot.runMotors(aOpMode, frontLeftMotorPower, frontRightMotorPower,
                backLeftMotorPower, backRightMotorPower);


    }


    public void driveRobotWithPowerFactor(vv_OpMode aOpMode, float powerFactor)
            throws InterruptedException {


        // takes the x and y cooridinates of the joystick and calculates the power for each motor that allows the robot to turn in that direction
        float forwardLeftPower = (Math.abs(aOpMode.gamepad1.left_stick_x) * aOpMode.gamepad1.left_stick_x) - ((Math.abs(aOpMode.gamepad1.left_stick_y)) * aOpMode.gamepad1.left_stick_y);
        float forwardRightPower = -(aOpMode.gamepad1.left_stick_x * Math.abs(aOpMode.gamepad1.left_stick_x)) - ((Math.abs(aOpMode.gamepad1.left_stick_y) * aOpMode.gamepad1.left_stick_y));
        float backLeftPower = -(aOpMode.gamepad1.left_stick_x * Math.abs(aOpMode.gamepad1.left_stick_x)) - ((Math.abs(aOpMode.gamepad1.left_stick_y) * aOpMode.gamepad1.left_stick_y));
        float backRightPower = (Math.abs(aOpMode.gamepad1.left_stick_x) * aOpMode.gamepad1.left_stick_x) - ((Math.abs(aOpMode.gamepad1.left_stick_y)) * aOpMode.gamepad1.left_stick_y);

        //Code to round powers when the driver wants to move diagonally
        if ((forwardLeftPower < .5f && forwardLeftPower > -.5f) && (forwardRightPower > .5f || forwardRightPower < .5f)) {
            forwardLeftPower = 0;
            backRightPower = 0;
        }
        if ((forwardRightPower < .5f && forwardRightPower > -.5f) && (forwardLeftPower > .5f || forwardLeftPower < .5f)) {
            forwardRightPower = 0;
            backLeftPower = 0;
        }
//        float forwardLeftPower = aOpMode.gamepad1.left_stick_y + aOpMode.gamepad1.right_stick_x + aOpMode.gamepad1.left_stick_x;
//        float backLeftPower = aOpMode.gamepad1.left_stick_y + aOpMode.gamepad1.right_stick_x - aOpMode.gamepad1.left_stick_x;
//        float forwardRightPower =  aOpMode.gamepad1.left_stick_y - aOpMode.gamepad1.right_stick_x - aOpMode.gamepad1.left_stick_x;
//        float backRightPower =  aOpMode.gamepad1.left_stick_y + aOpMode.gamepad1.right_stick_x + aOpMode.gamepad1.left_stick_x;

        //rotates or turns the robot
        if (Math.abs(aOpMode.gamepad1.right_stick_x) > vv_Constants.ANALOG_STICK_THRESHOLD) {
            runAllMotors(aOpMode, aOpMode.gamepad1.right_stick_x, -aOpMode.gamepad1.right_stick_x, aOpMode.gamepad1.right_stick_x, -aOpMode.gamepad1.right_stick_x);
        }
        //translates the robot using the Mecanum wheels
        else if (Math.abs(aOpMode.gamepad1.left_stick_x) > vv_Constants.ANALOG_STICK_THRESHOLD ||
                Math.abs(aOpMode.gamepad1.left_stick_y) > vv_Constants.ANALOG_STICK_THRESHOLD) {
            runAllMotors(aOpMode, (forwardLeftPower * powerFactor), (forwardRightPower * powerFactor), (backLeftPower * powerFactor), (backRightPower * powerFactor));
        } else {
            stopAllMotors(aOpMode);
        }
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
            robot.setPower(anOp, INTAKE_MOTOR, 0.0f);
            robot.setIntakeState(vv_Constants.IntakeStateEnum.Off);
        } else {
            robot.setPower(anOp, INTAKE_MOTOR, -vv_Constants.INTAKE_POWER); //TODO: Check negate
            robot.setIntakeState(vv_Constants.IntakeStateEnum.In);
        }
    }


    public void testMotor(vv_OpMode aOpMode, int motorName, float power, long duration)
            throws InterruptedException {
        aOpMode.DBG("in vvLib test Motor");
        robot.testMotor(aOpMode, motorName, power, duration);
    }

    public void testEncodedMotor(vv_OpMode aOpMode, int motorName, float power, int duration, int targetPosition)
            throws InterruptedException, vv_Robot.MotorStalledException {
        aOpMode.DBG("in testEncodedMotor");
        robot.testEncodedMotor(aOpMode, motorName, power, duration, targetPosition);
    }

    public void testSidewaysRight(vv_OpMode aOpMode, int duration)
            throws InterruptedException {


        //save old run mode
        DcMotor.RunMode oldRunMode = robot.getMotorMode(aOpMode, FRONT_LEFT_MOTOR);

        //set the new RunMode.

        robot.setMotorMode(aOpMode, FRONT_LEFT_MOTOR, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setMotorMode(aOpMode, FRONT_RIGHT_MOTOR, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setMotorMode(aOpMode, BACK_LEFT_MOTOR, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setMotorMode(aOpMode, BACK_RIGHT_MOTOR, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //wait just a bit for the changes to take effect.

        Thread.sleep(1000);


        robot.setPower(aOpMode, FRONT_LEFT_MOTOR, -0.4f);
        robot.setPower(aOpMode, FRONT_RIGHT_MOTOR, 0.4f);
        robot.setPower(aOpMode, BACK_LEFT_MOTOR, 0.4f);
        robot.setPower(aOpMode, BACK_RIGHT_MOTOR, -0.4f);

        aOpMode.reset_timer();
        while (aOpMode.time_elapsed() < duration) {
            //run till duration
            aOpMode.idle();
        }

        robot.stopBaseMotors(aOpMode);

        //restore the old Run Modes.

        robot.setMotorMode(aOpMode, FRONT_LEFT_MOTOR, oldRunMode);
        robot.setMotorMode(aOpMode, FRONT_RIGHT_MOTOR, oldRunMode);
        robot.setMotorMode(aOpMode, BACK_LEFT_MOTOR, oldRunMode);
        robot.setMotorMode(aOpMode, BACK_RIGHT_MOTOR, oldRunMode);

        //wait just a bit for the changes to take effect.

        Thread.sleep(1000);


        // TO MOVE SIDEWAYS RIGHT:
        /*
        front left - backwards
        back right - backwards
        front right - forwards
        back left - forwards
         */
    }

    public void decreaseLauncherPowerWithLimits(vv_OpMode aOpMode) throws InterruptedException,
            vv_Robot.MotorStalledException {
        int launcherPowerPosition = robot.getLauncherPowerPosition(aOpMode);
        if (launcherPowerPosition > LAUNCH_POWER_POSITION_MIN) {
            //decrement power.
            if ((launcherPowerPosition - LAUNCH_POWER_INCREMENT) > LAUNCH_POWER_POSITION_MIN) {
                launcherPowerPosition -= LAUNCH_POWER_INCREMENT;
            } else {
                launcherPowerPosition = LAUNCH_POWER_POSITION_MIN;
            }

            robot.setLauncherPowerPosition(aOpMode, launcherPowerPosition);
        }
    }

    public void increaseLauncherPowerWithLimits(vv_OpMode aOpMode) throws InterruptedException,
            vv_Robot.MotorStalledException {
        int launcherPowerPosition = robot.getLauncherPowerPosition(aOpMode);
        if (launcherPowerPosition < LAUNCH_POWER_POSITION_MAX) {
            //increment power.
            if ((launcherPowerPosition + LAUNCH_POWER_INCREMENT) < LAUNCH_POWER_POSITION_MAX) {
                launcherPowerPosition += LAUNCH_POWER_INCREMENT;
            } else {
                launcherPowerPosition = LAUNCH_POWER_POSITION_MAX;
            }
            robot.setLauncherPowerPosition(aOpMode, launcherPowerPosition);
        }
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

    public double getLauncherGatePosition(vv_OpMode aOpMode) {
        return robot.getLauncherGateServoPosition(aOpMode);
    }

    public void setLauncherGatePosition(vv_OpMode aOpMode, double position) {
        robot.setLauncherGateServoPosition(aOpMode, position);
    }


    public int getLauncherPowerPosition(vv_OpMode aOpMode) {
        return robot.getLauncherPowerPosition(aOpMode);
    }


    public double getFloorLightIntensity(vv_OpMode aOpMode) throws InterruptedException {
        return robot.getFloorLightIntensity(aOpMode);
    }

}
