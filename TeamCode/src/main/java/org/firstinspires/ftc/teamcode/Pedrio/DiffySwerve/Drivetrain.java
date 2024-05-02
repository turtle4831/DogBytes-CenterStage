package org.firstinspires.ftc.teamcode.Pedrio.DiffySwerve;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

public class Drivetrain {
    private Hardware instance = new Hardware();

    public Module leftModule = new Module(instance.leftModuleMotorA, instance.leftModuleMotorB);
    public Module rightModule = new Module(instance.rightModuleMotorA, instance.rightModuleMotorB);


    private ChassisSpeeds speeds;
    private double newAngle;
    private double leftModuleAngle;
    private double rightModuleAngle;


    public void drive(double x,double y, double z){
        //calculate the vector for modules to point at
        Vector2d vector = new Vector2d(x,y);

        //calculate the needed wheel angle and vector stuff

        //newAngle = vector.angle() - instance.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);//for field centric
        leftModuleAngle  = vector.angle() + (z * 35);
        rightModuleAngle = vector.angle() - (z * 35);

        leftModule.moveTo(vector.magnitude(), leftModuleAngle);
        rightModule.moveTo(vector.magnitude(), rightModuleAngle);


    }
    //auto stuff








}
