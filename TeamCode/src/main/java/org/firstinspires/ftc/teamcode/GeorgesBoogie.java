package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Disabled
//@Autonomous(name="Georges-BoggoelksnfLK /whoekfr")
public class GeorgesBoogie extends LinearOpMode {
    static String direction;
    public DcMotor FL_Motor;
    public DcMotor FR_Motor;
    public DcMotor BR_Motor;
    public DcMotor BL_Motor;



    public void encoderDrive(int speed, int leftTicks, int rightTicks) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            FL_Motor.setTargetPosition(leftTicks);
            FR_Motor.setTargetPosition(rightTicks);
            BL_Motor.setTargetPosition(leftTicks);
            BR_Motor.setTargetPosition(rightTicks);



            // Turn On RUN_TO_POSITION
            FL_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            //   runtime.reset();

            ((DcMotorEx)FL_Motor).setVelocity(speed);
            ((DcMotorEx)FR_Motor).setVelocity(speed);
            ((DcMotorEx)BL_Motor).setVelocity(speed);
            ((DcMotorEx)BR_Motor).setVelocity(speed);

            while(FR_Motor.isBusy() && FL_Motor.isBusy()) {
                telemetry.addData("FL pos", FL_Motor.getCurrentPosition());
                telemetry.addData("FR pos", FR_Motor.getCurrentPosition());
                telemetry.addData("BL pos", BL_Motor.getCurrentPosition());
                telemetry.addData("BR pos", BR_Motor.getCurrentPosition());
                telemetry.update();
            }

            FL_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            // Turn off RUN_TO_POSITION
            FL_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep((long) leftTicks + rightTicks);   // optional pause after each move.
            ((DcMotorEx) FL_Motor).setVelocity(0);
            ((DcMotorEx) FR_Motor).setVelocity(0);
            ((DcMotorEx) BL_Motor).setVelocity(0);
            ((DcMotorEx) BR_Motor).setVelocity(0);

        }
    }

    public void encoderDriveStrafe(int FrontLeft,int BackLeft,int FrontRight,int BackRight,int speed) {
        if (opModeIsActive()) {


            FL_Motor.setTargetPosition(FrontLeft);
            FR_Motor.setTargetPosition(FrontRight);
            BL_Motor.setTargetPosition(BackLeft);
            BR_Motor.setTargetPosition(BackRight);


            // Turn On RUN_TO_POSITION
            FL_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            //   runtime.reset();

            ((DcMotorEx) FL_Motor).setVelocity(speed);
            ((DcMotorEx) FR_Motor).setVelocity(speed);
            ((DcMotorEx) BL_Motor).setVelocity(speed);
            ((DcMotorEx) BR_Motor).setVelocity(speed);

            while(FR_Motor.isBusy() && FL_Motor.isBusy()) {
                telemetry.addData("FL pos", FL_Motor.getCurrentPosition());
                telemetry.addData("FR pos", FR_Motor.getCurrentPosition());
                telemetry.addData("BL pos", BL_Motor.getCurrentPosition());
                telemetry.addData("BR pos", BR_Motor.getCurrentPosition());
                telemetry.update();
            }
            FL_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION
            FL_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
            ((DcMotorEx) FL_Motor).setVelocity(0);
            ((DcMotorEx) FR_Motor).setVelocity(0);
            ((DcMotorEx) BL_Motor).setVelocity(0);
            ((DcMotorEx) BR_Motor).setVelocity(0);

        }
    }


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
