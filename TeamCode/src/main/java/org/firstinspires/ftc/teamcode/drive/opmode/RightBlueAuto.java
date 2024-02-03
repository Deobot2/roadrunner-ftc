package org.firstinspires.ftc.teamcode.drive.opmode;
import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.util.*;

//Maybe aren't needed
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;

@Autonomous
public class RightBlueAuto extends LinearOpMode {

    //Set up object detection
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/light_blue_cone.tflite";
    private static final String[] LABELS = {"Light Blue Cone"};
    //declare camera detection stuff
    private TfodProcessor tfod;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    //declare motors
    private DcMotor frontRight, frontLeft, backRight, backLeft;
    private DcMotor armControl;

    //declare end effector
    private CRServo grabberControl;

    //declare retention bar
    private Servo retentionBarControl;

    //declare imu
    private IMU imu;


    //For Switch Case
    private String stage = "detectionInit";

    private long startTime;

    private AprilTagDetection targetAprilTag;

    //Used to end the auto
    private boolean doneWithAuto = false;

    @Override
    public void runOpMode() throws InterruptedException {

        //int camera stuff
        initTfod();
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        imu = hardwareMap.get(IMU.class, "imu");
        updateOrientation();

        //init end effectors
        grabberControl = hardwareMap.get(CRServo.class, "grabberControl");

        //init retention bar
        retentionBarControl = hardwareMap.get(Servo.class, "retentionBarControl");

        //init motors
        armControl = hardwareMap.get(DcMotor.class, "armControl");

        telemetry.addLine("Init Done");

        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence Right = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(26.0)
                .turn(Math.toRadians(-90))
                .addDisplacementMarker(25, () -> retentionBarControl.setPosition(0.9))
                .addDisplacementMarker(60, () -> retentionBarControl.setPosition(0.5))
                .forward(7.75)
                .forward(-8.3)
                .turn(Math.toRadians(90))
                .forward(23.5)
                .turn(Math.toRadians(-90))
                .forward(-77.5)
                .turn(Math.toRadians(-180))
                .strafeLeft(20.5)
                .build();
        TrajectorySequence Left = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(26.0)
                .turn(Math.toRadians(90))
                .addDisplacementMarker(25, () -> retentionBarControl.setPosition(0.9))
                .addDisplacementMarker(60, () -> retentionBarControl.setPosition(0.5))
                .forward(7.75)
                .forward(-8.3)
                .turn(Math.toRadians(-90))
                .forward(23.5)
                .turn(Math.toRadians(-90))
                .forward(-79)
                .turn(Math.toRadians(-180))
                .strafeLeft(31.5)
                .build();
        TrajectorySequence Middle = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(32.5)
                .forward(-9.3)
                .turn(Math.toRadians(90))
                .addDisplacementMarker(20, () -> retentionBarControl.setPosition(0.9))
                .addDisplacementMarker(60, () -> retentionBarControl.setPosition(0.5))
                .forward(60)
                .forward(19.5)// add arm lower in the future
                .build();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)

        while(opModeIsActive() && !doneWithAuto) {
            telemetry.addData("Status", "Running");

            telemetry.addData("Stage",stage);
            telemetry.update();

            //Controls how long the code waits before checking if the detection model has recognized something or not
            long recogCheckWait = 5000;
            switch(stage){
                case "detectionInit":
                    startTime = System.currentTimeMillis();
                    stage = "detectionWait";
                    break;
                case "detectionWait":
                    long curTime = System.currentTimeMillis();
                    if(curTime - startTime > recogCheckWait){
                        stage = "detectionCheck";
                    }
                    break;
                case "detectionCheck":
                    List<Recognition> curRecogs = tfod.getRecognitions();
                    if(curRecogs.size() != 0){
                        /*double max = curRecogs.get(0).getConfidence();
                        int maxIndex = 0;
                        for(int i = 1;i < curRecogs.size();i++){
                            double newConfidence = curRecogs.get(i).getConfidence();
                            if(newConfidence > max){
                                max = newConfidence;
                                maxIndex = i;
                            }
                        }
                        Recognition maxRecog = curRecogs.get(maxIndex);
                        double xPosition = (maxRecog.getLeft() + maxRecog.getRight()) / 2;*/
                        Recognition recog = curRecogs.get(0);
                        double xPosition = (recog.getLeft() + recog.getRight()) / 2;
                        if(xPosition <= 300){
                            stage = "leftSpike";
                        }else{
                            stage = "middleSpike";
                        }
                    }else{
                        stage = "rightSpike";
                    }
                    break;
                case "middleSpike":
                    telemetry.addLine("We going middle");
                    telemetry.update();

                    retentionBarControl.setPosition(0.9);
                    drive.followTrajectorySequence(Middle);


                    stage = "parked";
                    break;
                case "rightSpike":
                    telemetry.addLine("We going right");
                    telemetry.update();

                    drive.followTrajectorySequence(Right);

                    stage = "parked";
                    break;
                case "leftSpike":
                    telemetry.addLine("We going left");
                    telemetry.update();

                    drive.followTrajectorySequence(Left);

                    stage = "parked";
                    break;
                case "park":
                    //Write code to park the robot
                    //I don't think this will be needed since we will already be inside the park zone when we place the pixel
                    stage = "parked";
                    break;
                case "parked":
                    resetWithoutEncoder();
                    frontRight.setPower(0);
                    frontLeft.setPower(0);
                    backRight.setPower(0);
                    backLeft.setPower(0);
                    //Check if this code works
                    //this should forever just loop
                    stage = "done";
                    break;
                case "done":
                    doneWithAuto = true;
                    break;
                default:
                    telemetry.addLine("Something went wrong");
                    telemetry.update();
                    break;
            }

            telemetryTfod();
            //last line

        }

        visionPortal.close();
    }

    private void resetWithoutEncoder(){
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //waitTime is in Milliseconds
    private void safeWait(long waitTime){
        long startTime = System.currentTimeMillis();
        long curTime = System.currentTimeMillis();
        while(curTime - startTime < waitTime){
            curTime = System.currentTimeMillis();
        }
    }
    //Updates the orientation of the robot for the IMU
    private void updateOrientation() {
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    //Initialize the TensorFlow Object Detection processor.
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        }
        //else {
        //    builder.setCamera(BuiltinCameraDirection.BACK);
        //}

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.90f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    //Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
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

}