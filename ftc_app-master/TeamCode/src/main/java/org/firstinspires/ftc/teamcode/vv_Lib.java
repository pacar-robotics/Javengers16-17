package org.firstinspires.ftc.teamcode;

import java.lang.Math;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by thomas on 9/25/2016.
 */

public class vv_Lib {
    private vv_Robot robot;

    public vv_Lib(vv_OpMode aOpMode) throws InterruptedException{
        robot = new vv_Robot();
        robot.init(aOpMode.hardwareMap, aOpMode);
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
        /*int turnDistance = (int) (angle * ((vv_Constants.ROBOT_TRACK * Math.PI) / 360)
                * (vv_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION / (vv_Constants.MECCANUM_WHEEL_DIAMETER * Math.PI)));
        */
        int turnDistance = (int) ((((angle / 360.0)*(vv_Constants.ROBOT_TRACK_DISTANCE * Math.PI)) /
                (vv_Constants.MECCANUM_WHEEL_DIAMETER * Math.PI))*(vv_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION * 4));

        switch (TurnDirection) {
            case Clockwise:
                robot.runRobotToPosition(aOpMode, power, -power, power, -power, turnDistance, -turnDistance, turnDistance, -turnDistance);
                break;
            case Counterclockwise:
                robot.runRobotToPosition(aOpMode, -power, power, -power, power, -turnDistance, turnDistance, -turnDistance, turnDistance);
                break;
        }
    }

    public void pushAButton(vv_OpMode aOpMode, vv_Constants.ButtonEnum buttonEnum) throws InterruptedException{
        robot.pushButton(aOpMode, buttonEnum);
        Thread.sleep(750);
    }

    public boolean senseTouch(vv_OpMode aOpMode) throws InterruptedException {
        return robot.getButtonTouchValue(aOpMode);
    }

    public void moveTillTouch(vv_OpMode aOpMode) throws InterruptedException {
        while (!senseTouch(aOpMode)) {
            robot.runMotors(aOpMode, .3f, .3f, .3f, .3f);
            aOpMode.idle();
        }
        robot.stopMotors(aOpMode);
    }

    //Moves robot forward with a distance supplied in centimeters and power between 0 and 1
    private void moveForwardToPosition(vv_OpMode aOpMode, float distance, float Power) throws InterruptedException {
        //we need to store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = (int) ((distance / (Math.PI * vv_Constants.MECCANUM_WHEEL_DIAMETER)) * vv_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        aOpMode.telemetryAddFormattedData("Motors", "Target Position", targetPosition);
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

    public void displayLineColorSensorLuminosity(vv_OpMode aOpMode) throws InterruptedException
    {
        aOpMode.telemetryAddData("Line Color Sensor", "Luminosity",":" +robot.getLineColorSensorAlpha(aOpMode));
        aOpMode.telemetryUpdate();
    }

    public void lineColorSensorOn(vv_OpMode aOpMode) throws InterruptedException
    {
        robot.enableLineColorSensorLED(aOpMode);
    }

    public void lineColorSensorOff(vv_OpMode aOpMode) throws InterruptedException
    {
        robot.disableLineColorSensorLED(aOpMode);
    }

    /*
    public int lineColorCalibration(vv_OpMode aOpMode) throws InterruptedException
    {
        int whiteLuminosityMinimum = robot.getLineColorSensorAlpha(aOpMode);
        return whiteLuminosityMinimum;
    }

*/
    public void moveUntilLine(vv_OpMode aOpMode) throws InterruptedException
    {

        while(robot.getLineColorSensorAlpha(aOpMode) < vv_Constants.LUMINOSITY_MINIMUM_WHITE)
        {
            robot.runMotors(aOpMode, -.3f, .3f, .3f, -.3f);
            aOpMode.idle();
        }
        robot.stopMotors(aOpMode);
        Thread.sleep(500);
    }

    public void turnUsingGyro(vv_OpMode aOpMode, int turnDegrees, vv_Constants.TurnDirectionEnum turnDirectionEnum) throws InterruptedException {
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

        /*
        turnDegrees = Math.abs(turnDegrees) - vv_Constants.GYRO_ERROR_MARGIN;
        if(turnDirectionEnum == vv_Constants.TurnDirectionEnum.Counterclockwise)
        {
            turnDegrees *= -1;
        }
*/

        while (Math.abs(zValue) < Math.abs(turnDegrees) && (System.currentTimeMillis() - startTime) < vv_Constants.MAX_MOTOR_LOOP_TIME) {

            //calculate proportional power to be used in turn. This starts off being 1 and drops off as the turn completes.
            float turnPower = (Math.abs(turnDegrees) - Math.abs(zValue)) / Math.abs(turnDegrees);

            //check for range of motor power values.

            if (turnPower < vv_Constants.MIN_TURN_SPEED) {
                turnPower = vv_Constants.MIN_TURN_SPEED;
            }

            if (turnPower > vv_Constants.MAX_TURN_SPEED) {
                turnPower = vv_Constants.MAX_TURN_SPEED;
            }

            //adjust for direction of turn by examining the sign of the turn degrees
            if(turnDirectionEnum == vv_Constants.TurnDirectionEnum.Counterclockwise)
            {
                turnPower *= -1;
            }

            //set the velocities to be used.

            frontLeftMotorPower = turnPower;
            frontRightMotorPower = -1 * turnPower;
            backLeftMotorPower = turnPower;
            backRightMotorPower = -1 * turnPower;

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
        robot.stopMotors(aOpMode);

        aOpMode.telemetryAddData("After Turn", "Degrees", ":" +robot.getBaseGyroSensorIntegratedZValue(aOpMode));
        aOpMode.telemetryUpdate();
        Thread.sleep(1000);
    }
}
