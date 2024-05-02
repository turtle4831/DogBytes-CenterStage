package org.firstinspires.ftc.teamcode.Pedrio.DiffySwerve;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

@Config
public class Hardware {

    private Hardware instance;


    public MotorEx leftModuleMotorA;
    public  MotorEx leftModuleMotorB;

    public MotorEx rightModuleMotorA;
    public MotorEx rightModuleMotorB;

    public IMU imu;

    public Hardware getInstance(){
        if(instance == null){
            instance = new Hardware();
        }
        return instance;
    }

    public void init(final HardwareMap hardwareMap){
        //initalize motors
        leftModuleMotorA = new MotorEx(hardwareMap,"leftModuleMotorA");
        leftModuleMotorB = new MotorEx(hardwareMap, "leftModuleMotorB");

        rightModuleMotorA = new MotorEx(hardwareMap, "rightModuleMotorA");
        rightModuleMotorB = new MotorEx(hardwareMap, "rightModuleMotorB");

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


    }
}
