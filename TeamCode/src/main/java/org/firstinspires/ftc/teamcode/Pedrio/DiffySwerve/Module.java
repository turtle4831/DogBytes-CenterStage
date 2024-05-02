package org.firstinspires.ftc.teamcode.Pedrio.DiffySwerve;

import static com.arcrobotics.ftclib.purepursuit.PurePursuitUtil.angleWrap;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class Module {
    private static final double TICKS_PER_INCH = 1221;//fix all constants
    private static final double MAX_RPS_TICKS = 0;
    final private double ENCODER_TICKS_PER_REV = 537.7;
    final double MAX_RPM = 1150;
    double degrees = ENCODER_TICKS_PER_REV / 360;
    final private  double WHEEL_RADIUS = 3;
    final private double MIN_VELOCITY = 0;

    final private double MAX_VELOCITY = (MAX_RPM / 60) * 1 * (2 * Math.PI);
    /*
        when both motors are going in the same direction with the same power the module turns
        when both motors are going in opposite directions with the same power the wheel turns
         */
    final private MotorEx motor1;
    final private MotorEx motor2;


    final double turn_kP = 0.01;
    final double drive_kP = 0.01;
    final private PIDController turnPid = new PIDController(turn_kP,0,0);

    final private PIDFController drivePid = new PIDFController(drive_kP,0,0,getMotorVelocityF(MAX_RPM / 60 * ENCODER_TICKS_PER_REV));


    final private PIDController motor1PID = new PIDController(0.01,0,0);//these should be the same
    final private PIDController motor2PID = new PIDController(0.01,0,0);
    private double currentModuleAngle = 0;

    private double angleA;
    private double angleB;
    private double optimzedAngle;

    private double motor_reverse_multiplier;
    private double currentPosTicks;
    private double currentPos;
    private double currentAngleTicks;
    private double currentAngle;
    private double oppAngle;
    private double angleFromTarget;
    private double oppAngleFromTarget;
    private double rPower;


    public Module(MotorEx motor1, MotorEx motor2){
        this.motor1 = motor1;//motor 1 controls the top gear
        this.motor2 = motor2;//motor 2 controls the bottom gear


    }

    public double getMotor1Location(){
        return motor1.getCurrentPosition();
    }
    public double getMotor2Location(){
        return motor2.getCurrentPosition();
    }

    public double getModuleHeading(){
        currentAngleTicks = (getMotor1Location() + getMotor2Location()) / 2;
        currentAngle = ticksToDegrees(currentAngleTicks);
        currentAngle = angleWrap(currentAngle);

        return currentAngle;
    }

    public double getWheelRotation(double currentPosA,double currentPosB){//calcs the change in position
        return ((currentPosA - getMotor1Location()) - (currentPosB - getMotor2Location())) / 2;
    }


    private double ticksToDegrees(double ticks){
        return ticks * degrees;
    }
    private double ticksToRads(double ticks){
        return ticksToDegrees(ticks) * (Math.PI /180);
    }
    public double getModuleDirectionDegrees(){
        return ticksToDegrees(getModuleHeading());
    }
    public double getModuleDirectionRadians(){
        return ticksToRads(getModuleHeading());
    }

    public double getModuleWheelDistance(){
        currentPosTicks = (getMotor1Location() - getMotor2Location()) / 2 ;
        currentPos = currentPosTicks / TICKS_PER_INCH;
        return currentPos;
    }


    public void calculate(Vector2d vector, double turn){
        // find if the angle can be optimized
        angleA = getModuleDirectionDegrees() + vector.angle() + (turn * 25);
        angleB = getModuleDirectionDegrees() - vector.angle() + (turn * 25);

        optimzedAngle = Math.min(angleA, angleB);//the number that is lowest is picked

        if (vector.angle() > 270  || vector.angle() > 0 && vector.angle() < 90){
            motor_reverse_multiplier = 1.0;
        }
        else{
            motor_reverse_multiplier = -1.0;
        }
        //get the angle from the vector object then set the pid to turn to it
        double output = turnPid.calculate(getModuleDirectionDegrees(),optimzedAngle);

        motor1.set(( (vector.magnitude() + turn) * (1 - output) * motor_reverse_multiplier  ) - output);
        motor2.set(( (vector.magnitude() - turn) * (1 - output) * motor_reverse_multiplier  ) + output);



    }


    public void moveTo(double velocity, double targetAngle){
        oppAngle = angleWrap(getModuleHeading() + 180);

        angleFromTarget = angleWrap(targetAngle - getModuleHeading());
        oppAngleFromTarget = angleWrap(targetAngle - oppAngle);

        if(Math.abs(angleFromTarget) > Math.abs(oppAngleFromTarget)){
            rPower = turnPid.calculate(getModuleHeading(), oppAngleFromTarget);
        }else{
            rPower = turnPid.calculate(getModuleHeading(), angleFromTarget);
        }

        if (Math.abs(oppAngleFromTarget) < 0.3){//margin of allowed error
            velocity = -velocity;
        }else if (Math.abs(angleFromTarget) > 0.3){
            velocity = 0;
        }

       //calc using the module pid
        motor1.setVelocity(((-velocity * 1) + rPower) * MAX_RPS_TICKS);
        motor2.setVelocity(((velocity * 1) + rPower) * MAX_RPS_TICKS);

    }




}
