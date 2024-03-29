package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
    Gamepad.RumbleEffect customRumbleEffect;

    public ElapsedTime time = new ElapsedTime();
    public DcMotor Rubber;
    public Servo Airplane;
    public Servo Wrist;
    public Servo Claw;
    boolean button;
    double slow = 1.0;




    public void speedControl(boolean on, double botHeading) {
        if (opModeIsActive()) {
            double[] motor = {FL_Motor.getPower(),FR_Motor.getPower(), BL_Motor.getPower(), BR_Motor.getPower()};
            if (on) {
                if ((botHeading >= 0.45 && botHeading <= 1.20) || (botHeading >= 2.25 && botHeading <= 3.20)) {
                    for (double v : motor) {
                        if (v >= 0.9) {
                            slow = 0.4;
                            //gamepad1.runRumbleEffect(customRumbleEffect);
                            time.reset();
                        } else if (time.seconds() > 1.5) {
                            slow = 1.0;
                            // gamepad1.stopRumble();,
                        }
                    }
                }
            }
        }
    }
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

        customRumbleEffect = new Gamepad.RumbleEffect.Builder().
                addStep(0.8, 0.8, 1500).build();
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        FL_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        BL_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        FR_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        BR_Motor.setDirection(DcMotorSimple.Direction.FORWARD);

        L_Slide.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        Airplane.setPosition(0.45);
        while(opModeIsActive()){
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//beginning of gamepad1

            //local variables for the motor power values. future me if u ever fix this replace axial and yaw
            double axial = -gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x ;


            //turning the power values to the sum of all the stick inputs
            double FRPower = axial + lateral + yaw;
            double FLPower = axial - lateral - yaw;
            double BRPower = axial - lateral + yaw;
            double BLPower = axial + lateral - yaw;

            //setting power to the power values set by the stick inputs
            FL_Motor.setPower(FLPower * slow);
            FR_Motor.setPower(FRPower * slow);
            BL_Motor.setPower(BLPower * slow);
            BR_Motor.setPower(BRPower * slow);

            if (gamepad1.b){ //airplane
                Airplane.setPosition(0);
            }
            if(gamepad1.a){
                Airplane.setPosition(0.45);
            }




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
                Wrist.setPosition(0.18);//wrist goes in
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
            if (gamepad2.dpad_left) {
                Rubber.setPower(-0.6);
                Intake.setPower(-0.4);
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

            if (gamepad2.dpad_right){
                slow = 0.5;
                time.reset();
                gamepad1.runRumbleEffect(customRumbleEffect);
            }

            if(time.seconds() > 2){
                slow = 1;
                gamepad1.stopRumble();
            }

            


//end of gamepad2
            telemetry.addData("FL_Motor Current Power", "%.2f", FLPower);   //gives telemetry for the FL Motor
            telemetry.addData("FR_Motor Current Power", "%.2f", FRPower);   //gives telemetry for the FR Motor
            telemetry.addData("BL_Motor Current Power", "%.2f", BLPower);   //gives telemetry for the BL Motor
            telemetry.addData("BR_Motor Current Power", "%.2f", BRPower);   //gives telemetry for the BR Motor
            telemetry.addData("Claw position", Claw.getPosition()); //gives the telemetry for the claws position
            telemetry.addData("Wrist position", Wrist.getPosition());//gives the telemetry for the wrists position
            telemetry.addData("Bot Heading: ", botHeading);
            telemetry.addData("Slow: ", slow);
            telemetry.addData("SpeedControl: ", button);

            telemetry.update();



        }
    }
}
