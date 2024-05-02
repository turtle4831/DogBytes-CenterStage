package org.firstinspires.ftc.teamcode.BattleBot.Components;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Spinner extends SubsystemBase {
    private final Motor Spinny;

    public Spinner(HardwareMap hmap){
        Spinny = new Motor(hmap,"spinny");
        Spinny.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }

    public void spin(double speed){
        Spinny.set(speed);
    }
}
