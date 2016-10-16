package org.firstinspires.ftc.teamcode;

import java.lang.Math;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.MotorConfiguration;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by thomas on 9/25/2016.
 */

public class vv_Lib {
    vv_Robot robot;

    public vv_Lib(vv_OpMode aOpMode) {
        robot = new vv_Robot();
        robot.init(aOpMode.hardwareMap);
    }

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


    public void turnUsingEncoders(vv_OpMode aOpMode, float power, float angle, vv_Constants.TurnDirectionEnum TurnDirection) {
        //code
    }

    public void pushAButton(vv_Constants.ButtonEnum buttonEnum) {
        robot.pushButton(buttonEnum);
    }

    public void turnUsingGyro(vv_OpMode aOpMode, float power, float angle, vv_Constants.TurnDirectionEnum TurnDirection) {
        // do we need direction?
        // absolute vs. relative turns
    }

    public int senseColor(vv_OpMode aOpMode, ColorSensor cs) {
        // three color sensors (left, right, line)
        // arguments?

        return 0;
    }

    public boolean senseTouch() throws InterruptedException {
        return robot.getButtonTouchValue();
    }

    public void moveTillTouch(vv_OpMode aOpMode) throws InterruptedException {
        while (!senseTouch()) {
            robot.runMotors(aOpMode, .3f, .3f, .3f, .3f);
        }
        robot.stopMotors();
    }

    public int moveTillColor(vv_OpMode aOpMode, ColorSensor cs) throws InterruptedException {
        return 0;
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
                VrdtargetPosition, Angle
        );
    }


    public void runAllMotors(vv_OpMode aOpMode, float FLPower, float FRPower, float BLPower, float BRPower) throws InterruptedException {
        robot.runMotors(aOpMode, FLPower, FRPower, BLPower, BRPower);
    }

    public void stopAllMotors() {
        robot.stopMotors();
    }

    //Moves robot forward with a distance supplied in centimeters and power between 0 and 1
    public void moveForward(float Power) throws InterruptedException {
        robot.runMotorsFB(Power);
    }

    //Moves robot backward with a distance supplied in centimeters and power between 0 and 1
    public void moveBackward(float Power) throws InterruptedException {
        robot.runMotorsFB(-Power);
    }

    public void moveSidewaysLeft(float Power) throws InterruptedException {
        robot.runMotorsSideways(Power);
    }

    public void moveSidewaysRight(float Power) throws InterruptedException {
        robot.runMotorsSideways(-Power);
    }

}
