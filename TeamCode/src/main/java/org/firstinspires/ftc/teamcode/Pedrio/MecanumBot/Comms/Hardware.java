package org.firstinspires.ftc.teamcode.Pedrio.MecanumBot.Comms;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Pedrio.MecanumBot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Pedrio.Util.PedrioSubsystem;
import org.firstinspires.ftc.teamcode.Pedrio.Util.RobotHardware;

@Config
public class Hardware extends RobotHardware {
    private Hardware instance = null;

    private HardwareMap hmap;

    public Motor FrontLeftMotor;
    public Motor FrontRightMotor;
    public Motor BackLeftMotor;
    public Motor BackRightMotor;

    public Motor.Encoder leftOdo;
    public Motor.Encoder rightOdo;
    public Motor.Encoder centerOdo;

    public Drivetrain drivetrain;

    public IMU imu;


    public Hardware getInstance(){
        if(instance == null){
            instance = new Hardware();
        }
        return instance;
    }

    public void init(final HardwareMap hmap){
        //all hardware pieces get initialized here
        this.hmap = hmap;
        this.FrontLeftMotor = hmap.get(Motor.class,"FrontLeft");
        this.FrontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.FrontRightMotor = hmap.get(Motor.class, "FrontRight");
        this.FrontRightMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        this.BackLeftMotor = hmap.get(Motor.class, "BackLeft");
        this.BackLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.BackRightMotor = hmap.get(Motor.class, "BackRight");
        this.BackRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.leftOdo = new Motor(hmap,"FrontLeft").encoder;
        this.rightOdo = new Motor(hmap, "FrontRight").encoder;
        this.centerOdo = new Motor(hmap, "BackLeft").encoder;

        this.imu = hmap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        this.imu.initialize(parameters);

        drivetrain = new Drivetrain();
        drivetrain.init();
    }

    public void periodic(){
        //put periodic functions for all subsystems in this block
        drivetrain.periodic();

    }





}
