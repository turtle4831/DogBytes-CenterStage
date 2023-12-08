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
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.CachingFocusControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;

import java.util.List;
@Autonomous(name = "Double Vision + encoder test")
public class visionEncoderAutoTest extends LinearOpMode {
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


    int id = 3;


    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal myVisionPortal;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/DogBytesSuperCoolRed.tflite"; // blue
    // change with blue stuff  private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/DogBytesSuperCoolModel.tflite";
    private static final String[] LABELS = { //these must be in training order
            // "RedTeam"
            "BlueTeamObj"
    };


    @Override
    public void runOpMode() {
        FL_Motor = hardwareMap.get(DcMotor.class, "FL_Motor");
        FR_Motor = hardwareMap.get(DcMotor.class, "FR_Motor");
        BL_Motor = hardwareMap.get(DcMotor.class, "BL_Motor");
        BR_Motor = hardwareMap.get(DcMotor.class, "BR_Motor");
        L_Slide =hardwareMap.get(DcMotor.class, "L_Slide");
        R_Slide = hardwareMap.get(DcMotor.class,"R_Slide");

        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Claw = hardwareMap.get(Servo.class, "Claw");

        //set the motor direction
        FL_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        BL_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        FR_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        BR_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        L_Slide.setDirection(DcMotorSimple.Direction.REVERSE);



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

                if (opModeInInit()) {
                    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
                    telemetry.addLine();
                    telemetry.addLine("----------------------------------------");
                }

                if (myVisionPortal.getProcessorEnabled(aprilTag)) {
                    // User instructions: Dpad left or Dpad right.
                    telemetry.addLine("Dpad Left to disable AprilTag");
                    telemetry.addLine();
                    telemetryAprilTag();
                } else {
                    telemetry.addLine("Dpad Right to enable AprilTag");
                }
                telemetry.addLine();
                telemetry.addLine("----------------------------------------");
                if (myVisionPortal.getProcessorEnabled(tfod)) {
                    telemetry.addLine("Dpad Down to disable TFOD");
                    telemetry.addLine();
                    telemetryTfod();
                } else {
                    telemetry.addLine("Dpad Up to enable TFOD");
                }

                // Push telemetry to the Driver Station.
                telemetry.update();
                List<Recognition> currentRecognitions = tfod.getRecognitions();
                for (Recognition recognition : currentRecognitions) {
                    if(recognition.getWidth() < 200 && recognition.getHeight() < 200 ){
                        this.x = (recognition.getLeft() + recognition.getRight()) / 2;
                        this.width = recognition.getWidth();
                        this.height = recognition.getHeight();
                    }else if (currentRecognitions.size() <= 1 ){
                        goLeft = true;
                    }


                }

                //for blue board side
                if (x >= 100 && x <= 300 && width >= 100 && width <= 200 && height >= 100 && height <= 200) {//middle
                    telemetry.addData("Going middle", x);
                    foundObj = true;
                    id = 2;
                } else if (x >= 350 && x <= 450 && width >= 100 && width <= 200 && height >= 100 && height <= 200) {// right
                    telemetry.addData("Going right", x);
                    foundObj = true;
                    id = 3;
                } else if(goLeft) {
                    telemetry.addData("Going left", x);
                    foundObj = true;
                    id = 1;
                }
                telemetry.update();

                if(foundObj){
                    if(id == 1){
                        //code for left
                        encoderDrive(1000,-500,500);
                        sleep(500);
                        sleep(500000);
                    }else if(id == 2 ){
                        //code for middle
                        encoderDrive(1000,2300,2300);
                        sleep(500);
                        sleep(500000);
                    }else if(id == 3){
                        //code for right
                        encoderDrive(300,400,400);
                        sleep(500);
                        encoderDrive(300,-500,500);
                        sleep(500);
                        encoderDrive(300,300,300);
                        sleep(500);
                        sleep(500000);
                    }
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
                .setModelAspectRatio(1.0)


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
    public void motorSleep(){
        FL_Motor.setTargetPosition(FL_Motor.getCurrentPosition());
        FR_Motor.setTargetPosition(FR_Motor.getCurrentPosition());
        BL_Motor.setTargetPosition(BL_Motor.getCurrentPosition());
        BR_Motor.setTargetPosition(BR_Motor.getCurrentPosition());

        ((DcMotorEx)FL_Motor).setVelocity(0);
        ((DcMotorEx)FR_Motor).setVelocity(0);
        ((DcMotorEx)BL_Motor).setVelocity(0);
        ((DcMotorEx)BR_Motor).setVelocity(0);

        FL_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (FL_Motor.getCurrentPosition() == FL_Motor.getTargetPosition()){
            FL_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


    }
    public void encoderDrive(int speed, int leftTicks, int rightTicks) { //encoder drive function

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


                // Turn off RUN_TO_POSITION
                FL_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FR_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BR_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BL_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                FL_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FR_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BR_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BL_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(250);   // optional pause after each move.
                ((DcMotorEx) FL_Motor).setVelocity(0);
                ((DcMotorEx) FR_Motor).setVelocity(0);
                ((DcMotorEx) BL_Motor).setVelocity(0);
                ((DcMotorEx) BR_Motor).setVelocity(0);
        }
    }
}