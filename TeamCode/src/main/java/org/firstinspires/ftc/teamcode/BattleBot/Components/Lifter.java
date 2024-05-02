package org.firstinspires.ftc.teamcode.BattleBot.Components;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Lifter extends SubsystemBase {
    private final SimpleServo left;
    private final SimpleServo right;

    public Lifter(HardwareMap hmap){
        left = new SimpleServo(hmap,"leftServo",0,180, AngleUnit.DEGREES);
        right = new SimpleServo(hmap,"rightServo",0,180, AngleUnit.DEGREES);

        right.setInverted(true);

    }


    public void MoveToAngle(double newAngle){
        left.rotateByAngle(newAngle);
        right.rotateByAngle(newAngle);
    }

    public void MoveToTicks(double ticks){
        left.rotateBy(ticks);
        right.rotateBy(ticks);
    }
}
