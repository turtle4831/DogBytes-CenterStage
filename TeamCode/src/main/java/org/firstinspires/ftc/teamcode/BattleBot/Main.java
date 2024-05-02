package org.firstinspires.ftc.teamcode.BattleBot;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "BB-Tele")
public class Main extends OpMode {

    private Motor left;
    private Motor right;
    private Motor flip_flop;

    private Motor flippy_flop;

    private DifferentialDrive diffy;



    private GamepadEx driver1;
    private ToggleButtonReader toggle;


    @Override
    public void init() {

        left = new Motor(hardwareMap,"left");
        right = new Motor(hardwareMap,"right");

        right.setInverted(true);
        flip_flop = new Motor(hardwareMap,"flip");
        flippy_flop = new Motor(hardwareMap, "flip2");

        flippy_flop.setInverted(true);

        diffy = new DifferentialDrive(left,right);

        driver1 = new GamepadEx(gamepad1);

        toggle = new ToggleButtonReader(driver1, GamepadKeys.Button.A);
    }


    @Override
    public void loop() {
        diffy.tankDrive(
            driver1.getLeftY(),
                driver1.getRightY()
        );
        flip_flop.set(gamepad2.left_trigger - gamepad2.right_trigger);
        flippy_flop.set(gamepad2.left_trigger - gamepad2.right_trigger);

    }
}
