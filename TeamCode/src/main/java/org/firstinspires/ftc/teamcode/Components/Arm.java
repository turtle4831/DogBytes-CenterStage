package org.firstinspires.ftc.teamcode.Components;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm extends SubsystemBase {
    private final double Kp = 0.01; //tune these
    private final double Ki = 0.0;
    private final double Kd = 0.0;
    private final double Kf = 0.0;
    private DcMotor arm;
    private final PIDFController pid = new PIDFController(Kp,Ki,Kd,Kf);

    public Arm(final HardwareMap hMap){
        arm = hMap.get(DcMotor.class, "arm");
    }

    public double getArmPos(){
        return arm.getCurrentPosition();
    }

    public double output(double measuredPos, double wantedPos){
        return pid.calculate(measuredPos, wantedPos);
    }

    public void moveArmToPose(double newPose){
        arm.setPower(output(getArmPos(),newPose));
    }

    }
