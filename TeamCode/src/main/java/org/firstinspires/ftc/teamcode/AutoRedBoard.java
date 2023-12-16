package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
@Autonomous(name = "RedBoardPurplePark")
public class AutoRedBoard extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    double x;
    double height;
    double width;
    boolean foundObj = false;
    boolean goLeft = false;
    public DcMotor FL_Motor;
    public DcMotor FR_Motor;
    public DcMotor BR_Motor;
    public DcMotor BL_Motor;
    public DcMotor L_Slide;
    public DcMotor R_Slide;
    public Servo Wrist;
    public Servo Claw;

    public DcMotor Intake;
    int id = 3;
    boolean notFound = true;


    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal myVisionPortal;
    //  private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/DogBytesSuperCoolRed.tflite"; // blue
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/DogBytesSuperCoolModel.tflite";
    private static final String[] LABELS = { //these must be in training order
            "RedTeam"
            //"BlueTeamObj"
    };


    @Override
    public void runOpMode() {
        FL_Motor = hardwareMap.get(DcMotor.class, "FL_Motor");
        FR_Motor = hardwareMap.get(DcMotor.class, "FR_Motor");
        BL_Motor = hardwareMap.get(DcMotor.class, "BL_Motor");
        BR_Motor = hardwareMap.get(DcMotor.class, "BR_Motor");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        L_Slide =hardwareMap.get(DcMotor.class, "L_Slide");
        R_Slide = hardwareMap.get(DcMotor.class,"R_Slide");

        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Claw = hardwareMap.get(Servo.class, "Claw");

        //set the motor direction
        FL_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        BL_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        FR_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        BR_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        L_Slide.setDirection(DcMotorSimple.Direction.REVERSE);
        R_Slide.setDirection(DcMotorSimple.Direction.FORWARD);

        Intake.setDirection(DcMotorSimple.Direction.REVERSE);




        //resets the encoders and starts them again cause i can
        FL_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        initDoubleVision();
        waitForStart();

        // This OpMode loops continuously, allowing the user to switch between
        // AprilTag and TensorFlow Object Detection (TFOD) image processors.
        if (opModeIsActive()) {
            while (!isStopRequested() && opModeIsActive()) {


                // Push telemetry to the Driver Station.

                telemetry.update();
                List<Recognition> currentRecognitions = tfod.getRecognitions();
                for (Recognition recognition : currentRecognitions) {
                    if ( recognition.getWidth() < 200 && recognition.getHeight() < 200 && recognition.getWidth() > 100 && recognition.getHeight() > 100 && (recognition.getLeft() + recognition.getRight()) / 2 < 490  && (recognition.getLeft() + recognition.getRight()) / 2 > 100) {
                        this.x = (recognition.getLeft() + recognition.getRight()) / 2;
                        this.width = recognition.getWidth();
                        this.height = recognition.getHeight();
                    }


                }


                //for blue board side
                if (x >= 100 && x <= 300 ) {//middle
                    telemetry.addData("Going middle", x);
                    foundObj = true;
                    id = 5;
                } else if (x >= 350 && x <= 480) {// right
                    telemetry.addData("Going right", x);
                    foundObj = true;
                    id = 6;
                } else {
                    telemetry.addData("Going left", x);
                    foundObj = true;
                    id = 4;
                }


                telemetry.update();

                if(foundObj){
                    myVisionPortal.setProcessorEnabled(tfod,false);
                    Claw.setPosition(1);
                    if(id == 4){
                        //code for left
                        encoderDrive(1000,700,-700); //goes forward 700 jahsdb
                        encoderDrive(600,-550,-550); //this actually turns to the left
                        encoderDrive(1000,350,-350); //also goes forward
                        Intake.setPower(-0.3);
                        encoderDrive(1000,-350,350);
                        Intake.setPower(0);
                        encoderDrive(600,550,550);
                        encoderDrive(1000,-700,700);
                        sleep(500);
                    }else if(id == 5 ){
                        //code for middle
                        encoderDrive(1500,1200,-1200);//make all the right values the opposite
                        Intake.setPower(-0.3);
                        encoderDrive(1500,-1200,1200);
                        Intake.setPower(0);


                    }else if(id == 6){
                        //code for right
                        encoderDriveStrafe(550,-550,-550,550,1000);
                        encoderDrive(1000,1100,-1100);
                        Intake.setPower(-0.3);
                        encoderDrive(1000,-1050,1050);
                        Intake.setPower(0);
                        encoderDriveStrafe(550,-550,-550,550,1000);
                    }
                    //after it returns to the starting position
                    encoderDrive(1000,150,-150);
                    encoderDriveStrafe(2500,-2500,-2500,2500,2500);
                    encoderDrive(1000,1000,1000);//turns left
                    encoderDrive(500,-500,500); //drives back some

                    while(notFound){
                        if(opModeIsActive()) {
                            FL_Motor.setPower(0.1);  //constanly strafes at a set speed
                            BL_Motor.setPower(-0.1);
                            FR_Motor.setPower(-0.1);
                            BR_Motor.setPower(0.1);

                            List<AprilTagDetection> currentDetections = aprilTag.getDetections(); //creates a list full of the current april tag detections
                            for (AprilTagDetection detection : currentDetections) { //for each of the detections
                                if (detection.metadata != null) { //if the aprilTag is a real tag
                                    if (detection.id == id) { //if its equal to the one we want it zeros the motor power and sends info to the driver hub
                                        telemetry.addData("Found ", detection.id);
                                        FL_Motor.setPower(0);
                                        BL_Motor.setPower(0);
                                        FR_Motor.setPower(0);
                                        BR_Motor.setPower(0);
                                        notFound = false;
                                    }
                                }
                            }
                        }
                    }
                    sleep(500);
                    telemetry.update();
                    encoderDrive(1500,2800,2800);
                    slideEncoder(1000,2000);
                    L_Slide.setPower(0.1);
                    R_Slide.setPower(0.1);
                    Wrist.setPosition(0.4);
                    encoderDrive(1000,-1000,1000);
                    Claw.setPosition(0.65);
                    sleep(1000);
                    L_Slide.setPower(0);
                    R_Slide.setPower(0);
                    encoderDriveStrafe(-1000,1000,1000,-1000,1500);
                    sleep(500);
                    sleep(500000000);
                }


                if (gamepad1.dpad_left) {
                    myVisionPortal.setProcessorEnabled(aprilTag, false);
                } else if (gamepad1.dpad_right) {
                    myVisionPortal.setProcessorEnabled(aprilTag, true);
                }
                if (gamepad1.dpad_down) {
                    myVisionPortal.setProcessorEnabled(tfod, false);
                } else if (gamepad1.dpad_up) {
                    myVisionPortal.setProcessorEnabled(tfod, true);
                }

                sleep(20);

            }   // end while loop

        }
    }// end method runOpMode()


    /**
     * Initialize AprilTag and TFOD.
     */
    private void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------

        tfod = new TfodProcessor.Builder()
                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.

                .setModelFileName(TFOD_MODEL_FILE)
                //  .setModelAssetName(TFOD_MODEL_ASSET)
                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                //.setModelInputSize(300)
                .setModelAspectRatio(16/9)

                .build();

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "camera1"))
                    .addProcessors(tfod, aprilTag)
                    .build();
        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }
    }   // end initDoubleVision()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()


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

            sleep(250);   // optional pause after each move.
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

}