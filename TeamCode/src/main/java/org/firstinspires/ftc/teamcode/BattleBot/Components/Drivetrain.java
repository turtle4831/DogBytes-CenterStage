package org.firstinspires.ftc.teamcode.BattleBot.Components;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {

    private DcMotor left;
    private DcMotor right;


    public Drivetrain(HardwareMap hmap){

        left = hmap.get(DcMotor.class, "left");
        right = hmap.get(DcMotor.class, "right");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double forward, double turn){
      double left_power = forward - turn;
      double right_power = forward + turn;

      left.setPower(left_power);
      right.setPower(right_power);

    }



}
