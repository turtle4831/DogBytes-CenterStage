package org.firstinspires.ftc.teamcode.Components;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;

public class Drivetrain extends SubsystemBase {
    public Drivetrain(HardwareMap hardwareMap){
        front_left = new Motor(hardwareMap, "front_left");
        front_right = new Motor(hardwareMap, "front_right");
        back_left = new Motor(hardwareMap, "back_left");
        back_right = new Motor(hardwareMap, "back_right");
        gyro = hardwareMap.get(IMU.class, "gyro");

        leftEncoder = new MotorEx(hardwareMap, "left odometer");
        rightEncoder = new MotorEx(hardwareMap, "right odometer");
        perpEncoder = new MotorEx(hardwareMap, "center odometer");

        leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        perpEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);


        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        gyro.initialize(parameters);

        odometry.updatePose();
    }
    private Motor front_left;
    private Motor front_right;
    private Motor back_left;
    private Motor back_right;

    private final MecanumDrive drive = new MecanumDrive(true,front_left,front_right,back_left,back_right);
    private final IMU gyro;
    private MotorEx leftEncoder, rightEncoder, perpEncoder;


    private HolonomicOdometry odometry = new HolonomicOdometry(
            leftEncoder::getDistance,
            rightEncoder::getDistance,
            perpEncoder::getDistance,
            TRACKWIDTH,
            CENTER_WHEEL_OFFSET
    );

    private static Pose2d pos;


    Translation2d m_frontLeftLocation =
            new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation =
            new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation =
            new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation =
            new Translation2d(-0.381, -0.381);

    // Creating my kinematics object using the wheel locations.
    MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics
            (
                    m_frontLeftLocation, m_frontRightLocation,
                    m_backLeftLocation, m_backRightLocation
            );

    //all variables under this need to be tuned
    public static final double TRACKWIDTH = 14.31;
    public static final double CENTER_WHEEL_OFFSET = 0.477;
    public static final double WHEEL_DIAMETER = 2.0;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;


    public double getGyroAngles(){

        return gyro != null ? gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) : 0;
    }


    public void updateOdo(){
        pos = odometry.getPose();
    }
    public Pose2d getPos(){
        return pos;
    }

    public double getXPos(){
        return pos.getX();
    }
    public double getYPos(){
        return pos.getY();
    }
    public double getZ(){
        return pos.getHeading();
    }

    public void FieldCentricDrive(double forward, double strafe, double turn){
        drive.driveFieldCentric(strafe,forward,turn,getGyroAngles());
    }

    public Trajectory generateTrajectory(){
        Pose2d start = new Pose2d(1,1, Rotation2d.fromDegrees(0));
        Pose2d end = new Pose2d(2,2, Rotation2d.fromDegrees(180));

        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();

        interiorWaypoints.add(new Translation2d(0,0 ));
        interiorWaypoints.add(new Translation2d(2,0 ));

        TrajectoryConfig config = new TrajectoryConfig(0.3,0.1);
        config.setReversed(false);

        return TrajectoryGenerator.generateTrajectory(
                start,
                interiorWaypoints,
                end,
                config);


        /*
        Trajectory.State goal = trajectory.sample(3.4); // sample the trajectory at 3.4 seconds from the beginning
        ChassisSpeeds adjustedSpeeds = controller.calculate(currentRobotPose, goal);

        ChassisSpeeds adjustedSpeeds = controller.calculate(currentRobotPose, goal);
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);
        double left = wheelSpeeds.leftMetersPerSecond;
        double right = wheelSpeeds.rightMetersPerSecond;

         */

    }



}


