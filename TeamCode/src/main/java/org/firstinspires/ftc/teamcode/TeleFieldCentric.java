package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "FieldCentricDrive Tele-op")
public class TeleFieldCentric extends LinearOpMode {
    public DcMotor Intake;

    public DcMotor L_Slide;
    public DcMotor R_Slide;
    public DcMotor Rubber;
    public Servo Airplane;
    public Servo Wrist;
    public Servo Claw;


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FL_Motor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BL_Motor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FR_Motor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BR_Motor");


        Intake = hardwareMap.get(DcMotor.class,"Intake");
        L_Slide =hardwareMap.get(DcMotor.class, "L_Slide");
        R_Slide = hardwareMap.get(DcMotor.class,"R_Slide");
        Rubber = hardwareMap.get(DcMotor.class,"Rubber");

        Airplane = hardwareMap.get(Servo.class, "Airplane");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Claw = hardwareMap.get(Servo.class, "Claw");

        L_Slide.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake.setDirection(DcMotorSimple.Direction.REVERSE);




        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();
        Airplane.setPosition(0.45);
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            double lift = gamepad2.right_trigger - gamepad2.left_trigger; //makes it so the slides take the sum of right and left trigger to give a power level
            L_Slide.setPower(lift);
            R_Slide.setPower(lift);


            //code for the wrist

            if(gamepad2.left_bumper){
                Wrist.setPosition(0.4); //wrist down half way
            }

            if (gamepad2.right_bumper){
                Wrist.setPosition(0.15);//wrist goes in
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


            if (gamepad2.b){ //airplane
                Airplane.setPosition(0);
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
        }
    }
}