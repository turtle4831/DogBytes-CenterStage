package org.firstinspires.ftc.teamcode;

import android.media.midi.MidiDevice;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Disabled
//@Autonomous(name="EncodersTestRR!11231231231")
public class AutoTestEncoders extends LinearOpMode {
    public DcMotor Intake;
    public DcMotor L_Slide;
    public DcMotor R_Slide;
    public DcMotor Rubber;
    public Servo Wrist;
    public Servo Claw;

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
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/DogBytesBlueV3.tflite"; // blue
    // private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/DogBytesSuperCoolModel.tflite";
    private static final String[] LABELS = { //these must be in training order
            //  "RedTeam"
            "BlueTeamObj"
    };

    public void slideEncoder(int speed, int distanceInTicks) {
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


            while (R_Slide.isBusy() && L_Slide.isBusy()) {
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

    public void ServoScore(){
        slideEncoder(2500,2400);
        sleep(500);
        Wrist.setPosition(0.4);
        sleep(500);
        Claw.setPosition(0.65);
    }
    public void ServoRetract(){
        Claw.setPosition(1);
        Wrist.setPosition(0.1);
        slideEncoder(1500,2000);
        sleep(500);
        Claw.setPosition(0.65);
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
    }   // end initDou


    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    double x;
    double height;
    double width;
    boolean foundObj = false;





    @Override
    public void runOpMode() {
        Intake = hardwareMap.get(DcMotor.class,"Intake");
        L_Slide =hardwareMap.get(DcMotor.class, "L_Slide");
        R_Slide = hardwareMap.get(DcMotor.class,"R_Slide");
        Rubber = hardwareMap.get(DcMotor.class,"Rubber");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Claw = hardwareMap.get(Servo.class, "Claw");


        L_Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R_Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initDoubleVision();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));

        drive.setPoseEstimate(startPose);


        Trajectory Shake = drive.trajectoryBuilder(startPose)
                .forward(-50)
                .back(-50)
                .build();


        Trajectory Left = drive.trajectoryBuilder(Shake.end()).strafeLeft(-100).build();

        //Trajectory Right = drive.trajectoryBuilder(Shake.end()).strafeRight(-100).forward(-600).build();

        //Trajectory Middle = drive.trajectoryBuilder(Shake.end()).forward(-1000).build();

        waitForStart();

        if(opModeIsActive() && !isStopRequested()) {




           Claw.setPosition(1);
           //detection
            telemetry.update();
            drive.followTrajectory(Shake);
            while(true){
                List<Recognition> currentRecognitions = tfod.getRecognitions();
                for (Recognition recognition : currentRecognitions) {
                    if ( recognition.getWidth() < 200 && recognition.getHeight() < 200 && recognition.getWidth() > 100 && recognition.getHeight() > 100 && (recognition.getLeft() + recognition.getRight()) / 2 < 490  && (recognition.getLeft() + recognition.getRight()) / 2 > 100) {
                        this.x = (recognition.getLeft() + recognition.getRight()) / 2;
                        this.width = recognition.getWidth();
                        this.height = recognition.getHeight();

                        //for blue board side
                        if (x >= 100 && x <= 300 ) {//middle
                            telemetry.addData("Going middle", x);
                            foundObj = true;
                            id = 2;
                        } else if (x >= 350 && x <= 480) {// right
                            telemetry.addData("Going right", x);
                            foundObj = true;
                            id = 3;
                        } else {
                            telemetry.addData("Going left", x);
                            foundObj = true;
                            id = 1;
                        }
                        break;
                    }
                }
                if (foundObj){
                    break;
                }
            }

            telemetry.update();

            switch (id) {
                case 1:
                    drive.followTrajectory(Left);
                    Intake.setPower(-0.5);

                    break;
                case 2:
                   // drive.followTrajectory(Middle);
                    Intake.setPower(-0.5);

                    break;
                case 3:
                    //drive.followTrajectory(Right);
                    Intake.setPower(-0.5);



                    break;
            }

            ServoScore();
            ServoRetract();
            sleep(500);
            sleep(500000000);
        }
    }
}
