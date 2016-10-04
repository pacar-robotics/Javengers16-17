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

public class vv_Lib
{
    public void moveWheels(LinearOpMode AOp, vv_Robot robot, float distance, float Power, vv_Constants.DirectionEnum Direction) throws InterruptedException
    {
        if (Direction == vv_Constants.DirectionEnum.Forward){
            // moving the robot forward
            moveForwardToPosition(AOp,robot, distance, Power);
        }else if (Direction == vv_Constants.DirectionEnum.Backward){
            // moving the robot forward
            moveBackwardToPosition(AOp,robot, distance, Power);
        }else if (Direction == vv_Constants.DirectionEnum.SidewaysLeft){
            // moving the robot forward
            moveSidewaysLeftToPosition(AOp,robot, distance, Power);
        }else if (Direction == vv_Constants.DirectionEnum.SidewaysRight){
            // moving the robot forward
            moveSidewaysRightToPosition(AOp,robot, distance, Power);
        }



        // code for moving forward, backward, sideways
    }


    public void turnUsingEncoders (LinearOpMode AOp, vv_Robot robot, float power, float angle, vv_Constants.TurnDirectionEnum TurnDirection)
    {
        //code
    }

    public void turnUsingGyro (LinearOpMode AOp, vv_Robot robot, float power, float angle, vv_Constants.TurnDirectionEnum TurnDirection)
    {
        // do we need direction?
        // absolute vs. relative turns
    }

    public int senseColor (LinearOpMode AOp, vv_Robot robot, ColorSensor cs)
    {
        // three color sensors (left, right, line)
        // arguments?

        return 0;
    }

    public boolean senseTouch (LinearOpMode AOp, vv_Robot robot, TouchSensor ts)
    {
        return true;
    }

    public int moveTillColor (LinearOpMode AOp, vv_Robot robot, ColorSensor cs)
    {
        // three color sensors (left, right, line)
        // arguments?

        return 0;
    }
    //Moves robot forward with a distance supplied in centimeters and power between 0 and 1
    private void moveForwardToPosition(LinearOpMode AOp, vv_Robot robot, float distance, float Power) throws InterruptedException{
        //we need to store the encoder target position
        int targetPosition = 0;
        //calculate target position from the input distance in cm
        targetPosition = (int)((distance / (Math.PI*vv_Constants.MECCANUM_WHEEL_DIAMETER))*vv_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //runs the robot to position
        robot.runRobotToPositionFB(AOp, targetPosition,Power);
    }
    //Moves robot backward with a distance supplied in centimeters and power between 0 and 1
    private void moveBackwardToPosition(LinearOpMode AOp, vv_Robot robot, float distance, float Power) throws InterruptedException{
        //we need to store the encoder target position
        int targetPosition = 0;
        //calculate target position from the input distance in cm
        targetPosition = -(int)((distance / (Math.PI*vv_Constants.MECCANUM_WHEEL_DIAMETER))*vv_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //runs the robot to position with negative power
        robot.runRobotToPositionFB(AOp, targetPosition,-Power);
    }
    private void moveSidewaysLeftToPosition(LinearOpMode AOp, vv_Robot robot, float distance, float Power) throws InterruptedException{
        //we need to store the encoder target position
        int targetPosition = 0;
        //calculate target position from the input distance in cm
        targetPosition = (int)((distance / (Math.PI*vv_Constants.MECCANUM_WHEEL_DIAMETER))*vv_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //runs the robot to position with negative power
        robot.runRobotToPositionSideways(AOp, targetPosition, Power);
    }
    private void moveSidewaysRightToPosition(LinearOpMode AOp, vv_Robot robot, float distance, float Power) throws InterruptedException{
        //we need to store the encoder target position
        int targetPosition = 0;
        //calculate target position from the input distance in cm
        targetPosition = -(int)((distance / (Math.PI*vv_Constants.MECCANUM_WHEEL_DIAMETER))*vv_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //runs the robot to position with negative power
        robot.runRobotToPositionSideways(AOp, targetPosition, -Power);
    }


    //DO NOT USE THIS METHOD
    //IT IS NOT COMPLETED
    public void moveAtAngle(vv_Robot robot, double distance, float Power, float Angle)
            throws InterruptedException{
        //we need to store the encoder target position
        int VldtargetPosition = 0;
        int VrdtargetPosition = 0;
        double Vld_distance = 0;
        double Vrd_distance = 0;

        float fl_Power = (float)((Math.pow(Math.sin(Angle),2.0) - (Math.pow(Math.cos(Angle),2.0))));

        float fr_Power = (float)(-(Math.pow(Math.sin(Angle),2.0) - (Math.pow(Math.cos(Angle),2.0))));

        float bl_Power = (float)(-(Math.pow(Math.sin(Angle),2.0) - (Math.pow(Math.cos(Angle),2.0))));

        float br_Power = (float)((Math.pow(Math.sin(Angle),2.0) - (Math.pow(Math.cos(Angle),2.0))));

        if (Angle>0 && Angle <45){
            Angle = 45 - (45 % Angle);
            Vld_distance = 1;
            Vrd_distance = 1;
            Vld_distance *= ((Angle*distance)/Math.sin(90));
            Vrd_distance *= (((90-Angle)*distance)/Math.sin(90));
        }
        else if (Angle>45 && Angle <90){
            Angle %= 45;
            Vld_distance = -1;
            Vrd_distance = 1;
            Vld_distance *= ((Angle*distance)/Math.sin(90));
            Vrd_distance *= (((90-Angle)*distance)/Math.sin(90));
        }
        else if (Angle>90 && Angle <135){
            Angle = 45 - (45 % Angle);
            Vld_distance = -1;
            Vrd_distance = 1;
            Vrd_distance *= ((Angle*distance)/Math.sin(90));
            Vld_distance *= (((90-Angle)*distance)/Math.sin(90));
        }else if (Angle>135 && Angle <180){
            Angle %= 45;
            Vld_distance = -1;
            Vrd_distance = -1;
            Vrd_distance *= ((Angle*distance)/Math.sin(90));
            Vld_distance *= (((90-Angle)*distance)/Math.sin(90));
        }
        else if (Angle>180 && Angle <225){
            Angle = 45 - (45 % Angle);
            Vld_distance = -1;
            Vrd_distance = -1;
            Vld_distance *= ((Angle*distance)/Math.sin(90));
            Vrd_distance *= (((90-Angle)*distance)/Math.sin(90));
        }else if (Angle>225 && Angle <270){
            Angle %= 45;
            Vld_distance = 1;
            Vrd_distance = -1;
            Vld_distance *= ((Angle*distance)/Math.sin(90));
            Vrd_distance *= (((90-Angle)*distance)/Math.sin(90));
        }
        else if (Angle>270 && Angle <315){
            Angle = 45 - (45 % Angle);
            Vld_distance = 1;
            Vrd_distance = -1;
            Vrd_distance *= ((Angle*distance)/Math.sin(90));
            Vld_distance *= (((90-Angle)*distance)/Math.sin(90));
        }
        else if (Angle>315 && Angle <360){
            Angle %= 45;
            Vld_distance = 1;
            Vrd_distance = 1;
            Vrd_distance *= ((Angle*distance)/Math.sin(90));
            Vld_distance *= (((90-Angle)*distance)/Math.sin(90));
        }


        VldtargetPosition = (int)((Vld_distance / (Math.PI*vv_Constants.MECCANUM_WHEEL_DIAMETER))*
                vv_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        VrdtargetPosition = (int)((Vrd_distance / (Math.PI*vv_Constants.MECCANUM_WHEEL_DIAMETER))*
                vv_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);





            //runs the robot to position with negative power
            robot.runRobotToPositionWithAngle(fl_Power*Power , fr_Power*Power , bl_Power*Power ,
                    br_Power*Power, VrdtargetPosition, VldtargetPosition, VldtargetPosition,
                    VrdtargetPosition, Angle);
    }





    //Moves robot forward with a distance supplied in centimeters and power between 0 and 1
    public void moveForward(vv_Robot robot, float Power) throws InterruptedException{
       robot.runMotorsFB(Power);
    }
    //Moves robot backward with a distance supplied in centimeters and power between 0 and 1
    public void moveBackward(vv_Robot robot,float Power) throws InterruptedException{
        robot.runMotorsFB(-Power);
    }
    public void moveSidewaysLeft(vv_Robot robot,float Power) throws InterruptedException{
        robot.runMotorsSideways(Power);
    }
    public void moveSidewaysRight(vv_Robot robot, float Power) throws InterruptedException{
        robot.runMotorsSideways(-Power);
    }

}
