package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "superdupercoolopmodethatlookssupercoolandhaslotsoffunctionality")
public class Tele extends LinearOpMode {
    //initiates all of the objects
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
            double yaw = -gamepad1.left_stick_y;  //for jake

            //double yaw = -gamepad1.right_stick_x * 0.7; for adam


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
              Airplane.setPosition(0.0);
            }
            if (gamepad1.b){//bring the servo back to last position
              Airplane.setPosition(1);
            }

//end of gamepad1
// beginning  of gamepad2
            Rubber.setPower(gamepad2.right_stick_y);
            //Intake.setPower(gamepad2.right_stick_y); //sets the intake power

            double lift = gamepad2.right_trigger - gamepad2.left_trigger; //makes it so the slides take the sum of right and left trigger to give a power level
            L_Slide.setPower(lift);
            R_Slide.setPower(lift);

            if(gamepad2.a) { //does something for the mech locks
                LeftMech.setPosition(1.0);
                RightMech.setPosition(1.0);
            }
            if(gamepad2.b){// does something for mech locks but the opposite of the other
                LeftMech.setPosition(0.0);
                RightMech.setPosition(0.0);
            }

            //code for the wrist
            if(gamepad2.dpad_up){
                Wrist.setPosition(0);   //wrist up
            }

            if(gamepad2.left_bumper){
                Wrist.setPosition(0.3); //wrist down half way
            }

            if (gamepad2.right_bumper){
                Wrist.setPosition(0);//wrist goes in
            }
            if (gamepad2.dpad_down){

            }
            if (gamepad2.y){
                Claw.setPosition(0.85);
            }
            if (gamepad2.x) {
                Claw.setPosition(1);//claw closes
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
