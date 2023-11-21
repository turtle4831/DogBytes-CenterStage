package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "BlueBoard")
public class AutoBlueBoard extends LinearOpMode {
    static String direction;
    final double WHEEL_DIAMETER = 96.0;
    final double PULSES_PER_REV = 1200;
    final double GEAR_REDUCTION = 2.0;
    final double COUNTS_PER_INCH = (PULSES_PER_REV * GEAR_REDUCTION) / (WHEEL_DIAMETER * 3.1415); //in inches
    public DcMotor FL_Motor;
    public DcMotor FR_Motor;
    public DcMotor BL_Motor;
    public DcMotor BR_Motor;
    public DcMotor L_Slide;
    public DcMotor R_Slide;
    public Servo Wrist;
    public Servo Claw;
    public ElapsedTime runtime = new ElapsedTime();

    public void encoderDrive(int speed, int leftTicks, int rightTicks) {

        if (opModeIsActive()) {
            FL_Motor.setTargetPosition(leftTicks);
            FR_Motor.setTargetPosition(rightTicks);
            BL_Motor.setTargetPosition(leftTicks);
            BR_Motor.setTargetPosition(rightTicks);

            FL_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ((DcMotorEx) FL_Motor).setVelocity(speed);
            ((DcMotorEx) FR_Motor).setVelocity(speed);
            ((DcMotorEx) BL_Motor).setVelocity(speed);
            ((DcMotorEx) BR_Motor).setVelocity(speed);

            if (FL_Motor.getCurrentPosition() == FL_Motor.getTargetPosition()) {

                FL_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FR_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BL_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BR_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                sleep(2500);
                ((DcMotorEx) FL_Motor).setVelocity(0);
                ((DcMotorEx) FR_Motor).setVelocity(0);
                ((DcMotorEx) BL_Motor).setVelocity(0);
                ((DcMotorEx) BR_Motor).setVelocity(0);
            }


        }
    }


    public void encoderDriveStrafe(int Frontleft, int Frontright, int Backleft, int Backright, int speed) {
        if (opModeIsActive()) {

            FL_Motor.setTargetPosition(Frontleft);
            FR_Motor.setTargetPosition(Frontright);
            BL_Motor.setTargetPosition(Backright);
            BR_Motor.setTargetPosition(Backright);

            FL_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ((DcMotorEx) FL_Motor).setVelocity(speed);
            ((DcMotorEx) FR_Motor).setVelocity(speed);
            ((DcMotorEx) BL_Motor).setVelocity(speed);
            ((DcMotorEx) BR_Motor).setVelocity(speed);

            if (FL_Motor.getCurrentPosition() == FL_Motor.getTargetPosition()) {

                FL_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FR_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BL_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BR_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                sleep(2500);
                ((DcMotorEx) FL_Motor).setVelocity(0);
                ((DcMotorEx) FR_Motor).setVelocity(0);
                ((DcMotorEx) BL_Motor).setVelocity(0);
                ((DcMotorEx) BR_Motor).setVelocity(0);


            }
        }
    }

    @Override
    public void runOpMode() {
        FL_Motor = hardwareMap.get(DcMotor.class, "FL_Motor");
        FR_Motor = hardwareMap.get(DcMotor.class, "FR_Motor");
        BL_Motor = hardwareMap.get(DcMotor.class, "BL_Motor");
        BR_Motor = hardwareMap.get(DcMotor.class, "BR_Motor");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Claw = hardwareMap.get(Servo.class, "Claw");
        L_Slide =hardwareMap.get(DcMotor.class, "L_Slide");
        R_Slide = hardwareMap.get(DcMotor.class,"R_Slide");


        L_Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R_Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        FL_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        FR_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        BL_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        BR_Motor.setDirection(DcMotorSimple.Direction.FORWARD);


        FL_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
            int[] motorLocations = new int[]{FL_Motor.getCurrentPosition(), FR_Motor.getCurrentPosition(),
                    BL_Motor.getCurrentPosition(), BR_Motor.getCurrentPosition()};


            encoderDriveStrafe(-450, 450, 450, -450, 2500);
            sleep(500);
            R_Slide.setPower(0.3);
            L_Slide.setPower(0.3);
            sleep(1500);
            R_Slide.setPower(0);
            L_Slide.setPower(0);
            sleep(500);
            Claw.setPosition(1);
            sleep(500);

            sleep(100000);


            for (int motorLocation : motorLocations) {
                telemetry.addData("Motor Location", motorLocation);
            }
            telemetry.addData("Which is the robot going", direction);
            telemetry.update();

        }
    }
}
