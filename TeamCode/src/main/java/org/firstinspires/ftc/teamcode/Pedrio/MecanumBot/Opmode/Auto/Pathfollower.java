package org.firstinspires.ftc.teamcode.Pedrio.MecanumBot.Opmode.Auto;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Pedrio.MecanumBot.Comms.Hardware;

public class Pathfollower {
    private Hardware robot = new Hardware().getInstance();

    private Pose2d currentPos;

    private PIDController xPid;
    private double xPidKp = 0.01;
    private PIDController yPid;
    private double yPidKp = 0.01;
    private PIDController zPid;
    private double zPidKp = 0.01;
    public void init(){
        currentPos = robot.drivetrain.getPose();
        xPid = new PIDController(xPidKp,0,0);
        yPid = new PIDController(yPidKp,0,0);
        zPid = new PIDController(zPidKp,0,0);

    }
    public void loop(){
        currentPos = robot.drivetrain.getPose();
    }
    public void calculatePath(Pose2d[] path){

    }
    public void calculate(Pose2d wantedPos){
        robot.drivetrain.driveFieldCentric(
                xPid.calculate(currentPos.getX(), wantedPos.getX()),
                yPid.calculate(currentPos.getY(), wantedPos.getY()),
                zPid.calculate(currentPos.getHeading(), wantedPos.getHeading()),
                currentPos.getHeading()
        );
    }

}
