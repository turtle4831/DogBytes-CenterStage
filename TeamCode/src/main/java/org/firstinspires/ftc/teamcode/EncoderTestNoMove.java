package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="EncodersTestNoMove")
public class EncoderTestNoMove extends LinearOpMode {
    static String direction;
    final double WHEEL_DIAMETER = 96.0;
    final double PULSES_PER_REV = 1200;
    final double GEAR_REDUCTION = 2.0;
    final double COUNTS_PER_INCH = (PULSES_PER_REV * GEAR_REDUCTION) / (WHEEL_DIAMETER * 3.1415); //in inches
    public DcMotor FL_Motor;
    public DcMotor FR_Motor;
    public DcMotor BR_Motor;
    public DcMotor BL_Motor;
    public ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        FL_Motor = hardwareMap.get(DcMotor.class, "FL_Motor");
        FR_Motor = hardwareMap.get(DcMotor.class, "FR_Motor");
        BL_Motor = hardwareMap.get(DcMotor.class, "BL_Motor");
        BR_Motor = hardwareMap.get(DcMotor.class, "BR_Motor");
        //set the motor direction
        FL_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        BL_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        FR_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        BR_Motor.setDirection(DcMotorSimple.Direction.FORWARD);



        //resets the encoders and starts them again cause i can
        FL_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();


        if (opModeIsActive()) {
            while (opModeIsActive()) {
                int[] motorLocations = new int[]{FL_Motor.getCurrentPosition(),FR_Motor.getCurrentPosition(),BL_Motor.getCurrentPosition(),BR_Motor.getCurrentPosition()};

                // adds the motor position to the telemetry
                for (int motorLocation : motorLocations) {
                    telemetry.addData("Motor locations", motorLocation);
                }
                telemetry.addData("Which way the robot is going", direction);
                telemetry.update();

            }
        }
    }
}
