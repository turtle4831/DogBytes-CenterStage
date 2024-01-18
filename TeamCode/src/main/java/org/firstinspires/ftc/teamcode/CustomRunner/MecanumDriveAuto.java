package org.firstinspires.ftc.teamcode.CustomRunner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

@Autonomous(name="MecanumAutoTest", group = "CustomStuff")
public class MecanumDriveAuto extends LinearOpMode {

    int frontleft;
    int backleft;
    int frontright;
    int backright;
    //init the motors + dead wheels
    public DcMotor LeftDead;//technically front left motor for when setting velocity
    public DcMotor RightDead;//technically front right motor for when setting velocity
    public DcMotor MiddleDead;//technically BackLeft motor for when setting velocity

    //add imu here when you find the id your comparing with your dead wheel values\

    public DcMotor BR_Motor;

    static int currentPosX = 0; //our current location to track the robot on the x axis
    static int currentPosY = 0; //our current location to track the robot on the y axis
    public  void splinyspline(double a, double c, double x, double y, int velocity,double acceleration, int acc) {
        if (opModeIsActive()) {
        /*
        the spline is equal to f(x) = 0.5(ax^3)+cx
        c controls the angle at which the spline starts
        a controls the rate at which the y value increases or the angle
        velocity is the original rate of speed while acceleration is the rate at which the velocity increases or decreases
        */
            ArrayList<Double> path = new ArrayList<>();
            currentPosX = (MiddleDead.getCurrentPosition());
            currentPosY = (LeftDead.getCurrentPosition() + RightDead.getCurrentPosition() / 2);

            //the dead wheels read 2500 counts per revolution
            //calculates the trajectory
            for (double i = 0; i < acc; i += 0.5) {
                path.add((double) c * x + 0.5 * (a * cube(x))); //calculates the x value and adds it to the que
            }
            for (double j = 0; j < path.size(); j += 1) {
                //after calculating the path it starts to move
                for(int i = 0; true; i++) {
                    i *= acceleration;

                    /*
                    double axial = gamepad1.right_stick_x;
                    double lateral = -gamepad1.left_stick_x;
                    double yaw = -gamepad1.left_stick_y;

                    double FRPower = axial + lateral + yaw;
                    double FLPower = axial - lateral - yaw;
                    double BRPower = axial - lateral + yaw;
                    double BLPower = axial + lateral - yaw;
                     */




                }
            }
        }
    }

    public void omnidirectionalDrive(){

    }

    public static double cube(double num){
        return num * num * num;
    }
    public void encoderDrive(int speed, int leftTicks, int rightTicks) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            LeftDead.setTargetPosition(leftTicks);
            RightDead.setTargetPosition(rightTicks);
            MiddleDead.setTargetPosition(leftTicks);
            BR_Motor.setTargetPosition(rightTicks);



            // Turn On RUN_TO_POSITION
            LeftDead.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightDead.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MiddleDead.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            //   runtime.reset();

            ((DcMotorEx)LeftDead).setVelocity(speed);
            ((DcMotorEx)RightDead).setVelocity(speed);
            ((DcMotorEx)MiddleDead).setVelocity(speed);
            ((DcMotorEx)BR_Motor).setVelocity(speed);

            while(LeftDead.isBusy() && RightDead.isBusy() && MiddleDead.isBusy()) {
                telemetry.addData("Left dead wheel pos", LeftDead.getCurrentPosition());
                telemetry.addData("Right dead wheel pos", RightDead.getCurrentPosition());
                telemetry.addData("Middle dead wheel pos", MiddleDead.getCurrentPosition());

                telemetry.update();
            }
            LeftDead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RightDead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            MiddleDead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            // Turn off RUN_TO_POSITION
            LeftDead.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightDead.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MiddleDead.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
            ((DcMotorEx) LeftDead).setVelocity(0);
            ((DcMotorEx) RightDead).setVelocity(0);
            ((DcMotorEx) MiddleDead).setVelocity(0);
            ((DcMotorEx) BR_Motor).setVelocity(0);

        }
    }
    public void encoderDriveStrafe(String direction, int length, int speed) {
        if (opModeIsActive()) {

            if(direction.equals("Left")){
                frontleft = length *-1;
                backleft = length;
                frontright = length;
                backright = length * -1;
            } else if (direction.equals("Right")) {
                frontleft = length;
                backleft = length *-1;
                frontright = length *-1;
                backright = length;
            }else{
                telemetry.addLine("Error wrong direction");
            }

            LeftDead.setTargetPosition(frontleft);
            RightDead.setTargetPosition(frontright);
            MiddleDead.setTargetPosition(backleft);
            BR_Motor.setTargetPosition(backright);

            LeftDead.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightDead.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MiddleDead.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ((DcMotorEx)LeftDead).setVelocity(speed);
            ((DcMotorEx)RightDead).setVelocity(speed);
            ((DcMotorEx)MiddleDead).setVelocity(speed);
            ((DcMotorEx)BR_Motor).setVelocity(speed);

            while(LeftDead.isBusy() && RightDead.isBusy() && MiddleDead.isBusy()) {
                telemetry.addData("Left dead wheel pos", LeftDead.getCurrentPosition());
                telemetry.addData("Right dead wheel pos", RightDead.getCurrentPosition());
                telemetry.addData("Middle dead wheel pos", MiddleDead.getCurrentPosition());

                telemetry.update();
            }

            LeftDead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RightDead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            MiddleDead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            // Turn off RUN_TO_POSITION
            LeftDead.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightDead.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MiddleDead.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            sleep(250);   // optional pause after each move.
            ((DcMotorEx) LeftDead).setVelocity(0);
            ((DcMotorEx) RightDead).setVelocity(0);
            ((DcMotorEx) MiddleDead).setVelocity(0);
            ((DcMotorEx) BR_Motor).setVelocity(0);


        }
    }



    @Override
    public void runOpMode(){

        LeftDead = hardwareMap.get(DcMotor.class, "FL_Motor");
        RightDead = hardwareMap.get(DcMotor.class, "FR_Motor");
        MiddleDead = hardwareMap.get(DcMotor.class, "BL_Motor");
        BR_Motor = hardwareMap.get(DcMotor.class, "BR_Motor");

        LeftDead.setDirection(DcMotorSimple.Direction.REVERSE);
        MiddleDead.setDirection(DcMotorSimple.Direction.REVERSE);
        RightDead.setDirection(DcMotorSimple.Direction.FORWARD);
        BR_Motor.setDirection(DcMotorSimple.Direction.FORWARD);

        LeftDead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MiddleDead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //
        splinyspline(0.2,1.2,13.0,7,2500,10.0,7);

    }
}
