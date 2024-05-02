package org.firstinspires.ftc.teamcode.Components;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Claw extends SubsystemBase {
    ServoEx claw;
    public Claw(HardwareMap hardwareMap){
        claw = new SimpleServo(
                hardwareMap, "servo_name", 0, 90,
                AngleUnit.DEGREES
        );
    }

    public void moveToAngle(double angle){
        claw.rotateByAngle(angle);
    }
    public void moveToPos(double newPos){
        claw.rotateBy(newPos);
    }
    public double getPos(){
        return claw.getPosition();
    }
    public double getAngle(){
        return claw.getAngle();
    }



}
