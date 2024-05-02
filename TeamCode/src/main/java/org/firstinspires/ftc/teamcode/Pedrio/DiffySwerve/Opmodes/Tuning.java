package org.firstinspires.ftc.teamcode.Pedrio.DiffySwerve.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Pedrio.DiffySwerve.Drivetrain;
import org.firstinspires.ftc.teamcode.Pedrio.DiffySwerve.Hardware;

@TeleOp(name = "tuning")
public class Tuning extends OpMode {
    private Hardware hardware = new Hardware();
    private Drivetrain drivetrain = new Drivetrain();
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        hardware = hardware.getInstance();

        telemetry.addData("LeftModuleMotor A ticks = ",hardware.leftModuleMotorA.getCurrentPosition());
        telemetry.addData("LeftModuleMotor B ticks = ",hardware.leftModuleMotorB.getCurrentPosition());

        telemetry.addData("leftModule Heading", drivetrain.leftModule.getModuleHeading());
        telemetry.addData("leftModule Distance", drivetrain.leftModule.getModuleWheelDistance());


    }
}
