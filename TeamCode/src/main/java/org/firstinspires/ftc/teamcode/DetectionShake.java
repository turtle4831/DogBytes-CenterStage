package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
@Disabled
//@tele_op(name = "Shake-Test")//this is for red audience
public class DetectionShake extends LinearOpMode {
    public DcMotor FL_Motor;
    public DcMotor FR_Motor;
    public DcMotor BL_Motor;
    public DcMotor BR_Motor;

    private ElapsedTime runtime = new ElapsedTime();
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    VisionPortal myVisionPortal;
    @SuppressLint("SdCardPath")
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/DogBytesSuperCoolModel.tflite";// use this if ur storing on the control hub
    private static final String[] LABELS = { //these must be in training order
            "RedTeam"
            //"BlueTeamObj"
    };
    private boolean Found;

    private int id;

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
                //.setIsModelQuantized(true)
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
    @SuppressLint("DefaultLocale")
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
            telemetry.update();
        }   // end for() loop

    }   // end method telemetryTfod()


    @Override
    public void runOpMode(){
        initDoubleVision();

        FL_Motor = hardwareMap.get(DcMotor.class, "FL_Motor");
        FR_Motor = hardwareMap.get(DcMotor.class, "FR_Motor");
        BL_Motor = hardwareMap.get(DcMotor.class, "BL_Motor");
        BR_Motor = hardwareMap.get(DcMotor.class, "BR_Motor");

        FL_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        BL_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        FR_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        BR_Motor.setDirection(DcMotorSimple.Direction.FORWARD);

        FL_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        waitForStart();
        encoderDrive(2500,250,250);
        while (!Found) {
            if (runtime.seconds() >=  10.0){ //if it takes george too long to detect he parks
                encoderDrive(1000,-100,-100);
                encoderDriveStrafe(4500,-4500,-4500,4500,2500);
                break;
            }
            encoderDrive(2500,200,-200);
            encoderDrive(2500,-200,200);
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("Time elapsed: ", runtime.seconds());
            for (Recognition recognition : currentRecognitions) {
                telemetryTfod();
                if (recognition.getWidth() < 200 && recognition.getHeight() < 200 && recognition.getWidth() > 100 && recognition.getHeight() > 100 && (recognition.getLeft() + recognition.getRight()) / 2 < 490 && (recognition.getLeft() + recognition.getRight()) / 2 > 100) {
                    double x  = (recognition.getLeft() + recognition.getRight()) / 2;
                    this.Found = true;

                    if (x >= 100 && x <= 300 ){
                        //middle id = 5
                       this.id = 5;
                    } else if (x >= 350 && x <= 480){
                        //right id = 6
                       this.id = 6;
                    }else{
                        //left id = 4
                        this.id = 4;
                    }
                    myVisionPortal.setProcessorEnabled(tfod,false);
                    // object is found
                }
            }
            telemetry.update();
        } //end of the Detection boogie
        telemetry.addData("The id is: ", id);
        telemetry.update();
        //george gets to moving frfr
        if(id > 0 ){
            if(id == 4){ //id for the left
                //code to score purple pixel on the left spike mark
            }
            if(id == 5){// id for the middle
                //code to score purple pixel on the middle spike mark
            }
            if(id == 6){//id for the right
                //code to score purple pixel on the right spike mark
            }
        }else{
            sleep(50000);
        }
    }
}
