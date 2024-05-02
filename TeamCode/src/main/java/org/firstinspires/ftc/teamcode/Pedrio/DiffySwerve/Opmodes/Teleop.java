package org.firstinspires.ftc.teamcode.Pedrio.DiffySwerve.Opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Pedrio.DiffySwerve.Drivetrain;

import java.util.List;

@TeleOp(name = "Diffy-Swerve-Test")
public class Teleop extends OpMode {
    private GamepadEx driver = new GamepadEx(gamepad1);
    private Drivetrain swerve = new Drivetrain();

    List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);


    @Override
    public void init() {
        for(LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void loop() {
        swerve.drive(
                driver.getLeftX(),
                driver.getLeftY(),
                driver.getRightX()
        );

    }
}
