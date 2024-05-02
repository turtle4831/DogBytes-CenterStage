package org.firstinspires.ftc.teamcode.Pedrio.MecanumBot.Subsystems;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Pedrio.MecanumBot.Comms.Hardware;
import org.firstinspires.ftc.teamcode.Pedrio.Util.PedrioSubsystem;

public class Drivetrain extends PedrioSubsystem {
    final private double TicksToInches = 12; //when doing this for your robot make sure to change this
    private Hardware robot = new Hardware().getInstance();
    private MecanumDrive drive = new MecanumDrive(robot.FrontLeftMotor,robot.FrontRightMotor,robot.BackLeftMotor,robot.BackRightMotor);


    private MecanumDriveKinematics kine = new MecanumDriveKinematics(
            new Translation2d(0,0),//change this to match your robots measurement in meters
            new Translation2d(0,0),
            new Translation2d(0,0),
            new Translation2d(0,0)
    );

    private HolonomicOdometry odo = new HolonomicOdometry(
            robot.leftOdo::getDistance,
            robot.rightOdo::getDistance,
            robot.centerOdo::getDistance,
            12,  //change this to match your robots
            0//and this they both in meters
    );

    public OdometrySubsystem odoSub = new OdometrySubsystem(odo);

    public void driveFieldCentric(double x, double y, double turn, double gyroAngle){
        drive.driveFieldCentric(x,y,turn,gyroAngle);
    }
    public Pose2d getPose(){
        return odo.getPose();
    }

    public double getRawIMUHeadingDegrees(){
        return robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public double getRawIMUHeadingRadians(){
        return robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getOdoHeadingDegrees(){
        return odoSub.getPose().getHeading();
    }


    @Override
    public void init() {
        robot.leftOdo.setDistancePerPulse(TicksToInches);
        robot.rightOdo.setDistancePerPulse(TicksToInches);
        robot.centerOdo.setDistancePerPulse(TicksToInches);
    }

    @Override
    public void periodic() {
        odo.update(
                robot.leftOdo.getDistance(),
                robot.rightOdo.getDistance(),
                robot.centerOdo.getDistance()
        );
    }
}
