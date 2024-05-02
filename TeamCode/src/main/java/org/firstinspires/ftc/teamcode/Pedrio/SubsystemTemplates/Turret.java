package org.firstinspires.ftc.teamcode.Pedrio.SubsystemTemplates;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret extends SubsystemBase {
    final private Motor swivel;
    final private boolean Encoder;
    final private double  minDistance;
    final private double maxDistance;

    private PIDController pid;

    public Turret(HardwareMap hmap, String swivelMotorName, boolean Encoder, double minDistance, double maxDistance, double kp, double ki, double kd){

        swivel = hmap.get(Motor.class, swivelMotorName);
        this.Encoder = Encoder;
        this.minDistance = minDistance;
        this.maxDistance = maxDistance;

        pid.setPID(kp, ki, kd);

    }

    public double EncoderTest(){
        //returns distance in ticks if the encoder is turned on
        if (!Encoder){
            return 0;
        }
        return swivel.getCurrentPosition();
    }

    public double GetEncoder(){
        if (!Encoder){
            return 0;
        }
        return swivel.getCurrentPosition();
    }

    public void MoveToPosEnc(double newPos){
        if (!Encoder){
            return;
        }
        //measured the desired pos
        swivel.set(pid.calculate(swivel.getCurrentPosition(), newPos));
    }

    public void MoveToPossiblePos(double newPos){
        if (!Encoder){
            return;
        }
        double PosPos = scale_number(newPos,minDistance,maxDistance,-1.0,1.0);

        swivel.set(pid.calculate(swivel.getCurrentPosition(), PosPos));

    }

    public void JoystickControlled(double JoystickInput){
        if (!Encoder){
            return;
        }
        double PosPos = scale_number(JoystickInput,minDistance,maxDistance,-1.0,1.0);

        swivel.set(pid.calculate(swivel.getCurrentPosition(), PosPos));
    }

    public void JoystickControlledNoEncoder(double JoystickInput, double ScaleFactor){
        if (abs(ScaleFactor) == 0 ){
            ScaleFactor = 0.3;
        }
        swivel.set(JoystickInput * ScaleFactor);
    }

    private double scale_number(double unscaled, double toMin,double toMax, double fromMin, double fromMax){
        return (toMax - toMin) * (unscaled - fromMin)/ (fromMax - fromMin) + toMin;
    }


}
