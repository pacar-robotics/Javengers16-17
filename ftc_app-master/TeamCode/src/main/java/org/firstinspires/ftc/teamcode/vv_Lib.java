package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by thomas on 9/25/2016.
 */

public class vv_Lib {
    vv_Robot robot;


    public vv_Lib(vv_OpMode aOpMode) throws InterruptedException{
        robot = new vv_Robot();
        robot.init(aOpMode.hardwareMap, aOpMode);
    }

    /**
     * Controls the Mecanum wheels on the robot using the two Joysticks.
     *
     * gamepad1.left_stick_x = rotates the robot using a dual wheel turn gamepad1.left_stick_y =
     * translates the robot in all directions
     */
    public void processDriveRobotWithPowerFactor(vv_OpMode aOpMode, float powerFactor) throws InterruptedException {
        //            if(gamepad1.left_stick_x > vv_Constants.ANALOG_STICK_THRESHOLD) {
//                vvLib.moveSidewaysRight(robot, gamepad1.left_stick_x);
//            }else if(gamepad1.left_stick_x < -vv_Constants.ANALOG_STICK_THRESHOLD) {
//                vvLib.moveSidewaysLeft(robot, -gamepad1.left_stick_x);
//            }else if(gamepad1.left_stick_y > vv_Constants.ANALOG_STICK_THRESHOLD) {
//                vvLib.moveForward(robot, -gamepad1.left_stick_y);
//            }else if(gamepad1.left_stick_y < -vv_Constants.ANALOG_STICK_THRESHOLD) {
//                vvLib.moveBackward(robot, gamepad1.left_stick_y);
//            }else{
//                robot.stopMotors();
//            }

        // takes the x and y cooridinates of the joystick and calculates the power for each motor that allows the robot to turn in that direction
        float forwardLeftPower = (Math.abs(aOpMode.gamepad1.left_stick_x) * aOpMode.gamepad1.left_stick_x) - ((Math.abs(aOpMode.gamepad1.left_stick_y)) * aOpMode.gamepad1.left_stick_y);
        float forwardRightPower = -(aOpMode.gamepad1.left_stick_x * Math.abs(aOpMode.gamepad1.left_stick_x)) - ((Math.abs(aOpMode.gamepad1.left_stick_y) * aOpMode.gamepad1.left_stick_y));
        float backLeftPower = -(aOpMode.gamepad1.left_stick_x * Math.abs(aOpMode.gamepad1.left_stick_x)) - ((Math.abs(aOpMode.gamepad1.left_stick_y) * aOpMode.gamepad1.left_stick_y));
        float backRightPower = (Math.abs(aOpMode.gamepad1.left_stick_x) * aOpMode.gamepad1.left_stick_x) - ((Math.abs(aOpMode.gamepad1.left_stick_y)) * aOpMode.gamepad1.left_stick_y);


        //rotates/turns the robot
        if (Math.abs(aOpMode.gamepad1.right_stick_x) > vv_Constants.ANALOG_STICK_THRESHOLD) {
            runAllMotors(aOpMode, aOpMode.gamepad1.right_stick_x, -aOpMode.gamepad1.right_stick_x, aOpMode.gamepad1.right_stick_x, -aOpMode.gamepad1.right_stick_x);
        }
        //translates the robot using the mecanum wheels
        else if (Math.abs(aOpMode.gamepad1.left_stick_x) > vv_Constants.ANALOG_STICK_THRESHOLD ||
                Math.abs(aOpMode.gamepad1.left_stick_y) > vv_Constants.ANALOG_STICK_THRESHOLD) {
            runAllMotors(aOpMode, (forwardLeftPower * powerFactor), (forwardRightPower * powerFactor), (backLeftPower * powerFactor), (backRightPower * powerFactor));
        } else {
            stopAllMotors(aOpMode);
        }
    }


    /**
     * moveWheels method
     * @param aOpMode - object of vv_OpMode class
     * @param distance - in centimeters
     * @param Power - float
     * @param Direction - forward, backward, sideways left, or sideways right
     * @throws InterruptedException
     */
    public void moveWheels(vv_OpMode aOpMode, float distance, float Power, vv_Constants.DirectionEnum Direction) throws InterruptedException {
        if (Direction == vv_Constants.DirectionEnum.Forward) {
            // moving the robot forward
            moveForwardToPosition(aOpMode, distance, Power);
        } else if (Direction == vv_Constants.DirectionEnum.Backward) {
            // moving the robot forward
            moveBackwardToPosition(aOpMode, distance, Power);
        } else if (Direction == vv_Constants.DirectionEnum.SidewaysLeft) {
            // moving the robot forward
            moveSidewaysLeftToPosition(aOpMode, distance, Power);
        } else if (Direction == vv_Constants.DirectionEnum.SidewaysRight) {
            // moving the robot forward
            moveSidewaysRightToPosition(aOpMode, distance, Power);
        }
        // code for moving forward, backward, sideways
    }

    public void setupShot(vv_OpMode aOpMode) throws InterruptedException
    {

       robot.setMotorMode(aOpMode, vv_Constants.MotorEnum.armMotor, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(!robot.isArmAtLimit(aOpMode)){
            robot.setPower(aOpMode, vv_Constants.MotorEnum.armMotor, 1.0f);
        }
        robot.setPower(aOpMode, vv_Constants.MotorEnum.armMotor, 0.0f);

        Thread.sleep(100);

    }

    public void shootBall(vv_OpMode aOpMode) throws InterruptedException
    {
        robot.setMotorMode(aOpMode, vv_Constants.MotorEnum.armMotor, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.setPower(aOpMode, vv_Constants.MotorEnum.armMotor, 1.0f);

        Thread.sleep(500);

        robot.setPower(aOpMode, vv_Constants.MotorEnum.armMotor, 0.0f);
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
    public void turnUsingEncoders (vv_OpMode aOpMode, float power, float angle, vv_Constants.TurnDirectionEnum TurnDirection)throws InterruptedException
    {
        int turnDistance = (int) (angle * ((vv_Constants.ROBOT_TRACK * Math.PI) / 360)
                * (vv_Constants.TETRIX_MOTOR_ENCODER_COUNTS_PER_REVOLUTION / (vv_Constants.MECCANUM_WHEEL_DIAMETER * Math.PI)));

        switch (TurnDirection) {
            case Clockwise:
                robot.runRobotToPosition(aOpMode, power, -power, power, -power, turnDistance, turnDistance, turnDistance, turnDistance);
                break;
            case Counterclockwise:
                robot.runRobotToPosition(aOpMode, -power, power, -power, power, turnDistance, turnDistance, turnDistance, turnDistance);
                break;
        }
    }

    public void pushAButton(vv_OpMode aOpMode, vv_Constants.BeaconServoStateEnum buttonEnum) {
        robot.pushButton(aOpMode, buttonEnum);
    }

    public void turnUsingGyro(vv_OpMode aOpMode, float power, float angle, vv_Constants.TurnDirectionEnum TurnDirection) {
        // do we need direction?
        // absolute vs. relative turns
    }

    public boolean senseTouch(vv_OpMode aOpMode) throws InterruptedException {
        return robot.getButtonTouchValue(aOpMode);
    }

    public void moveTillTouch(vv_OpMode aOpMode) throws InterruptedException {
        while (!senseTouch(aOpMode)) {
            robot.runMotors(aOpMode, .3f, .3f, .3f, .3f);
        }
        robot.stopMotors(aOpMode);
    }

    /**
     * Method that moves robot until the color white is detected
     * Used to stop at white line when going from first to second beacon
     * @param aOpMode - object of vv_OpMode class
     * @param cs
     * @throws InterruptedException
     */
    public void moveTillColor(vv_OpMode aOpMode, ColorSensor cs) throws InterruptedException {
        while (!((cs.red() < 235) || (cs.green() < 235) || (cs.blue() < 235))) {
            aOpMode.telemetryAddFormattedData("test: ", "cs red value: ", cs.red());
            aOpMode.telemetryAddFormattedData("test1: ", "cs green value: ", cs.green());
            aOpMode.telemetryAddFormattedData("test2: ", "cs blue value: ", cs.blue());
            moveSidewaysLeft(aOpMode, .3f);
        }
    }

    //Moves robot forward with a distance supplied in centimeters and power between 0 and 1
    private void moveForwardToPosition(vv_OpMode aOpMode, float distance, float Power) throws InterruptedException {
        //we need to store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = (int) ((distance / (Math.PI * vv_Constants.MECCANUM_WHEEL_DIAMETER)) * vv_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //runs the robot to position
        robot.runRobotToPositionFB(aOpMode, targetPosition, Power);
    }

    //Moves robot backward with a distance supplied in centimeters and power between 0 and 1
    private void moveBackwardToPosition(vv_OpMode aOpMode, float distance, float Power) throws InterruptedException {
        //we need to store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = -(int) ((distance / (Math.PI * vv_Constants.MECCANUM_WHEEL_DIAMETER)) * vv_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //runs the robot to position with negative power
        robot.runRobotToPositionFB(aOpMode, targetPosition, -Power);
    }

    private void moveSidewaysLeftToPosition(vv_OpMode aOpMode, float distance, float Power) throws InterruptedException {
        //we need to store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = (int) ((distance / (Math.PI * vv_Constants.MECCANUM_WHEEL_DIAMETER)) * vv_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //runs the robot to position with negative power
        robot.runRobotToPositionSideways(aOpMode, targetPosition, Power);
    }

    private void moveSidewaysRightToPosition(vv_OpMode aOpMode, float distance, float Power) throws InterruptedException {
        //we need to store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = -(int) ((distance / (Math.PI * vv_Constants.MECCANUM_WHEEL_DIAMETER)) * vv_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //runs the robot to position with negative power
        robot.runRobotToPositionSideways(aOpMode, targetPosition, -Power);
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


        VldtargetPosition = (int) ((Vld_distance / (Math.PI * vv_Constants.MECCANUM_WHEEL_DIAMETER)) *
                vv_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        VrdtargetPosition = (int) ((Vrd_distance / (Math.PI * vv_Constants.MECCANUM_WHEEL_DIAMETER)) *
                vv_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);


        //runs the robot to position with negative power
        robot.runRobotToPositionWithAngle(aOpMode, fl_Power * Power, fr_Power * Power, bl_Power * Power,
                br_Power * Power, VrdtargetPosition, VldtargetPosition, VldtargetPosition,
                VrdtargetPosition, Angle);
    }


    public void runAllMotors(vv_OpMode aOpMode, float FLPower, float FRPower, float BLPower, float BRPower) throws InterruptedException {
        robot.runMotors(aOpMode, FLPower, FRPower, BLPower, BRPower);
    }

    public void stopAllMotors(vv_OpMode aOpMode) {
        robot.stopMotors(aOpMode);
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
        robot.runMotorsSideways(aOpMode, Power);
    }

    public void moveSidewaysRight(vv_OpMode aOpMode, float Power) throws InterruptedException {
        robot.runMotorsSideways(aOpMode, -Power);
    }

    /**
     * Runs the Cap Ball Lift to either the scoring position or the rest position
     *
     * @param anOp         an object of vv_OpMode
     * @param capBallState the two states in which the Cap Ball Lift can be in: Scoring and Rest; is
     *                     the state in which the user wants the lift to go to
     * @param Power        the power value which is the applied to the Cap Ball Lift motor
     */
    public void moveCapBallLiftToPosition(vv_OpMode anOp, vv_Constants.CapBallStateEnum capBallState, float Power) throws InterruptedException {
        switch (capBallState) {
            //moves the CapBall Lift to Rest Position
            case Rest:
                robot.moveCapBallLift(anOp, vv_Constants.CAP_BALL_LIFT_REST, Power);
                robot.CapBallState = vv_Constants.CapBallStateEnum.Rest;
                break;
            //Move the CapBall Lift to the Scoring Position
            case Scoring_Position:
                robot.moveCapBallLift(anOp, vv_Constants.CAP_BALL_LIFT_SCORE, Power);
                robot.CapBallState = vv_Constants.CapBallStateEnum.Scoring_Position;
                break;
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
        if (robot.BallCollectorState == vv_Constants.BallCollectorStateEnum.Outtake ||
                robot.BallCollectorState == vv_Constants.BallCollectorStateEnum.Intake) {
            robot.setPowerToBallCollector(anOp, 0.0f);
            robot.BallCollectorState = vv_Constants.BallCollectorStateEnum.Off;
        } else {
            robot.setPowerToBallCollector(anOp, vv_Constants.BALL_COLLECTOR_POWER); //TODO: Check negate
            robot.BallCollectorState = vv_Constants.BallCollectorStateEnum.Outtake;
        }
    }

    /**
     * Toggles the power of the Ball Collector Motor to either off or to the power required to
     * intake depending on the current state of the motor
     *
     * @param anOp an object of vv_OpMode
     */
    public void toggleIntake(vv_OpMode anOp) throws InterruptedException {
        // if the current ball collector state is Outtake or Intake, turn the motor off
        // else set the ball collector motor power to the intake power
        if (robot.BallCollectorState == vv_Constants.BallCollectorStateEnum.Intake ||
                robot.BallCollectorState == vv_Constants.BallCollectorStateEnum.Outtake) {
            robot.setPowerToBallCollector(anOp, 0.0f);
            robot.BallCollectorState = vv_Constants.BallCollectorStateEnum.Off;
        } else {
            robot.setPowerToBallCollector(anOp, -vv_Constants.BALL_COLLECTOR_POWER); //TODO: Check negate
            robot.BallCollectorState = vv_Constants.BallCollectorStateEnum.Intake;
        }
    }

    /**
     * Changes the powerFactor of the wheels depending on the state of the Cap Ball Lift
     *
     * @return either 1.0 or CAP_BALL_SCORE_POWER_FACTOR
     */
    public float powerFactorBasedOnCapBall() {
        //if the cap ball lift is at rest, set the power factor to 1
        //else set the cap ball lift to the power factor needed to score the cap ball
        if (robot.CapBallState == vv_Constants.CapBallStateEnum.Rest) {
            return 1.0f;
        } else {
            return vv_Constants.CAP_BALL_SCORE_POWER_FACTOR;
        }
    }
}
