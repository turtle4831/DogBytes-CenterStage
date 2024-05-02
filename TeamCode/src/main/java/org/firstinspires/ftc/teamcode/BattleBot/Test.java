package org.firstinspires.ftc.teamcode.BattleBot;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "testServo")
public class Test extends OpMode {
private ServoEx servo;
    @Override
    public void init() {
        servo = new SimpleServo(hardwareMap,"servo",0,90);
    }

    @Override
    public void loop() {
    if(gamepad1.a){
        servo.turnToAngle(90);
    } else if (gamepad1.b) {
        servo.turnToAngle(0);
    }

    }
}
