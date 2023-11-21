package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Auto Pre-Positioning test")
public class AutoPrePositioning extends LinearOpMode {

    public DcMotor FL_Motor;
    public DcMotor FR_Motor;
    public DcMotor BR_Motor;
    public DcMotor BL_Motor;
    public DistanceSensor Distance_Left;
    public DistanceSensor Distance_Right;
    public DistanceSensor Distance_Back;


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

            if (FL_Motor.getCurrentPosition() == FL_Motor.getTargetPosition()) {


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

            if (FL_Motor.getCurrentPosition() == FL_Motor.getTargetPosition()) {


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
    }


    @Override
    public void runOpMode() {
        FL_Motor = hardwareMap.get(DcMotor.class, "FL_Motor");
        FR_Motor = hardwareMap.get(DcMotor.class, "FR_Motor");
        BL_Motor = hardwareMap.get(DcMotor.class, "BL_Motor");
        BR_Motor = hardwareMap.get(DcMotor.class, "BR_Motor");
        Distance_Left = hardwareMap.get(DistanceSensor.class,"Distance_Left");
        Distance_Right = hardwareMap.get(DistanceSensor.class,"Distance_Right");
        Distance_Back = hardwareMap.get(DistanceSensor.class,"Distance_Back");
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
                //call a sleep for 500 ms after every encoder drive function
                while(true){//localization loop for board blue and audience red
                    while(Distance_Right.getDistance(DistanceUnit.INCH) > 5){ //makes the robot be in the correct right position
                        int RightDistance = (int)Distance_Right.getDistance(DistanceUnit.INCH); //strafes to the right until its in the correct position
                        encoderDriveStrafe(RightDistance,-RightDistance,-RightDistance,RightDistance,1000);
                    }
                    while(Distance_Back.getDistance(DistanceUnit.INCH) < 3){ //goes forwards until the distance is less than 3 inches
                        int BackDistance = (int) Distance_Back.getDistance(DistanceUnit.INCH);
                        encoderDrive(1000,BackDistance,BackDistance);
                    }
                    if (Distance_Right.getDistance(DistanceUnit.INCH) > 5 && Distance_Back.getDistance(DistanceUnit.INCH) < 3){
                        break;
                    }
                }



                sleep(100000000);

                // adds the motor position to the telemetry
                for (int motorLocation : motorLocations) {
                    telemetry.addData("Motor locations", motorLocation);
                }
                telemetry.update();

            }
        }
    }
}
