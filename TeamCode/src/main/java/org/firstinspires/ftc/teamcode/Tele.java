package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "superdupercoolopmodethatlookssupercoolandhaslotsoffunctionality")
public class Tele extends LinearOpMode {
    //initiates all of the objects
    boolean Fullpower = false;
    public DcMotor FL_Motor;
    public DcMotor FR_Motor;
    public DcMotor BL_Motor;
    public DcMotor BR_Motor;
    public DcMotor Intake;
    public DcMotor L_Slide;
    public DcMotor R_Slide;
    public DcMotor Rubber;
    public Servo Airplane;
    public Servo Wrist;
    public Servo Claw;
    public Servo LeftMech;
    public Servo RightMech;
    public void slideEncoder(int speed, int distanceInTicks){
        if (opModeIsActive()) {


            L_Slide.setTargetPosition(distanceInTicks);
            R_Slide.setTargetPosition(distanceInTicks);


            // Turn On RUN_TO_POSITION
            L_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            R_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            //   runtime.reset();

            ((DcMotorEx) L_Slide).setVelocity(speed);
            ((DcMotorEx) R_Slide).setVelocity(speed);



            while(R_Slide.isBusy() && L_Slide.isBusy()) {
                telemetry.addData("Left Slide pos", L_Slide.getCurrentPosition());
                telemetry.addData("Right Slide pos", R_Slide.getCurrentPosition());

                telemetry.update();
            }
            L_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            R_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION
            L_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            R_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            sleep(250);   // optional pause after each move.
            ((DcMotorEx) L_Slide).setVelocity(0);
            ((DcMotorEx) R_Slide).setVelocity(0);


        }
    }

    @Override
    public void runOpMode(){
        FL_Motor = hardwareMap.get(DcMotor.class,"FL_Motor");
        FR_Motor = hardwareMap.get(DcMotor.class,"FR_Motor");
        BL_Motor = hardwareMap.get(DcMotor.class,"BL_Motor");
        BR_Motor = hardwareMap.get(DcMotor.class,"BR_Motor");
        Intake = hardwareMap.get(DcMotor.class,"Intake");
        L_Slide =hardwareMap.get(DcMotor.class, "L_Slide");
        R_Slide = hardwareMap.get(DcMotor.class,"R_Slide");
        Rubber = hardwareMap.get(DcMotor.class,"Rubber");

        FL_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        L_Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R_Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Airplane = hardwareMap.get(Servo.class, "Airplane");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Claw = hardwareMap.get(Servo.class, "Claw");
        LeftMech = hardwareMap.get(Servo.class,"LeftMech");
        RightMech = hardwareMap.get(Servo.class,"RightMech");




        FL_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        BL_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        FR_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        BR_Motor.setDirection(DcMotorSimple.Direction.FORWARD);

        L_Slide.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        while(opModeIsActive()){
//beginning of gamepad1

            //local variables for the motor power values. future me if u ever fix this replace axial and yaw
            double axial = -gamepad1.right_stick_x;
            double lateral = -gamepad1.left_stick_x;
            double yaw = -gamepad1.left_stick_y;


            //turning the power values to the sum of all the stick inputs
            double FRPower = axial + lateral + yaw;
            double FLPower = axial - lateral - yaw;
            double BRPower = axial - lateral + yaw;
            double BLPower = axial + lateral - yaw;

            //setting power to the power values set by the stick inputs
            FL_Motor.setPower(FLPower);
            FR_Motor.setPower(FRPower);
            BL_Motor.setPower(BLPower);
            BR_Motor.setPower(BRPower);

            if (gamepad1.a){//for shooting airplane
              Airplane.setPosition(0.3);
            }
            if (gamepad1.b){//bring the servo back to last position
              Airplane.setPosition(0.5);
            }
            //add more airplane stuff in player 1




//end of gamepad1
// beginning  of gamepad2



            double lift = gamepad2.right_trigger - gamepad2.left_trigger; //makes it so the slides take the sum of right and left trigger to give a power level
            L_Slide.setPower(lift);
            R_Slide.setPower(lift);


            //code for the wrist

            if(gamepad2.left_bumper){
                Wrist.setPosition(0.4); //wrist down half way
            }

            if (gamepad2.right_bumper){
                Wrist.setPosition(0.1);//wrist goes in
            }
            if (gamepad2.y){
                Claw.setPosition(0.65);
            }
            if (gamepad2.x) {
                Claw.setPosition(1);//claw closes
            }

           //code for intake/conveyor
            if(gamepad2.dpad_up) {
                Intake.setPower(1.0);
                Rubber.setPower(0.55); //for testing when the locking is in change to 6
            }
            if(gamepad2.dpad_down){
                Intake.setPower(0);
                Rubber.setPower(0);
            }
            if (gamepad2.dpad_left){
                Rubber.setPower(-0.6);
            }
            if (gamepad2.dpad_right){
                Rubber.setPower(0.75);
            }
            if (gamepad2.a){
                L_Slide.setPower(-0.45);
                R_Slide.setPower(-0.45);
                while(true){
                    if(gamepad2.b){
                        R_Slide.setPower(0);
                        L_Slide.setPower(0);
                        break;
                    }else{
                        sleep(10);
                    }
                }
            }



            


//end of gamepad2
            telemetry.addData("FL_Motor Current Power", "%.2f", FLPower);   //gives telemetry for the FL Motor
            telemetry.addData("FR_Motor Current Power", "%.2f", FRPower);   //gives telemetry for the FR Motor
            telemetry.addData("BL_Motor Current Power", "%.2f", BLPower);   //gives telemetry for the BL Motor
            telemetry.addData("BR_Motor Current Power", "%.2f", BRPower);   //gives telemetry for the BR Motor
            telemetry.addData("Claw position", Claw.getPosition()); //gives the telemetry for the claws position
            telemetry.addData("Wrist position", Wrist.getPosition());//gives the telemetry for the wrists position

            telemetry.update();



        }
    }
}
