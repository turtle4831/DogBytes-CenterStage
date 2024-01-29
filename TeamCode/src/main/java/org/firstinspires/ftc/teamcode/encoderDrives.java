package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.nio.channels.FileLock;
@Disabled
//@Autonomous(name = "AutoTestGeorge")
public class encoderDrives extends LinearOpMode {

    public  DcMotor LeftDead;
    public DcMotor RightDead;
    public  DcMotor MiddleDead;
    public  DcMotor FR_Motor;

    public  void EncoderDrive(int Left,int Right, int speed){
        LeftDead.setTargetPosition(-Left);
        RightDead.setTargetPosition(-Right);
        MiddleDead.setTargetPosition(0);

        LeftDead.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightDead.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MiddleDead.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ((DcMotorEx)LeftDead).setVelocity(-speed);
        ((DcMotorEx)RightDead).setVelocity(-speed);
        ((DcMotorEx)MiddleDead).setVelocity(speed);
        ((DcMotorEx)FR_Motor).setVelocity(speed);

        while(LeftDead.isBusy() && RightDead.isBusy()){
            if(LeftDead.getCurrentPosition() == LeftDead.getCurrentPosition() && RightDead.getCurrentPosition() == RightDead.getTargetPosition()){
                ((DcMotorEx)LeftDead).setVelocity(0);
                ((DcMotorEx)RightDead).setVelocity(0);
                ((DcMotorEx)MiddleDead).setVelocity(0);
                ((DcMotorEx)FR_Motor).setVelocity(0);
            }

        }
        LeftDead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MiddleDead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftDead.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightDead.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MiddleDead.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public boolean motorsActive(){
        return LeftDead.isBusy() && RightDead.isBusy() && MiddleDead.isBusy() && FR_Motor.isBusy();
    }


    public void testEncoderMoveForward(int distance, int speed){
        if (opModeIsActive()) {
            ((DcMotorEx) LeftDead).setVelocity(speed);
            ((DcMotorEx) RightDead).setVelocity(speed);
            ((DcMotorEx) MiddleDead).setVelocity(speed);
            ((DcMotorEx) FR_Motor).setVelocity(speed);
            sleep(40);
            while (LeftDead.getCurrentPosition() != distance && RightDead.getCurrentPosition() != distance) {

            }
            ((DcMotorEx) LeftDead).setVelocity(0);
            ((DcMotorEx) RightDead).setVelocity(0);
            ((DcMotorEx) MiddleDead).setVelocity(0);
            ((DcMotorEx) FR_Motor).setVelocity(0);
        }
    }

    @Override
    public void runOpMode(){


        LeftDead = hardwareMap.get(DcMotor.class, "FL_Motor");
        RightDead = hardwareMap.get(DcMotor.class, "BR_Motor");
        MiddleDead = hardwareMap.get(DcMotor.class, "BL_Motor");
        FR_Motor = hardwareMap.get(DcMotor.class,"FR_Motor");

        LeftDead.setDirection(DcMotorSimple.Direction.FORWARD);
        RightDead.setDirection(DcMotorSimple.Direction.FORWARD);
        MiddleDead.setDirection(DcMotorSimple.Direction.REVERSE);
        FR_Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftDead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MiddleDead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        if (opModeIsActive()){
            testEncoderMoveForward(1500,250);
        }
    }
}
