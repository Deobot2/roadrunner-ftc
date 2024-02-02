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
public class LeftRedAuto extends LinearOpMode {

    //Set up object detection
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/red_cone_FPNLite.tflite";
    private static final String[] LABELS = {
            "Red Cone",
    };
    //declare camera detection stuff
    private TfodProcessor tfod;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    //declare motors
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor armControl;
    //declare end effectors
    private CRServo grabberControl;

    //declare retention bar
    private Servo retentionBarControl;

    //declare imu
    private IMU imu;

    //save which cone position was detected so proper april tag can be done
    private int coneLocation = -1;

    //For Switch Case
    private String stage = "detectionInit";

    private long startTime;

    private AprilTagDetection targetAprilTag;

    //Used to end the auto
    private boolean doneWithAuto = false;

    //Given 3inch diameter mechanum wheels, 5000 ticks goes ~91 in
    //Therefore ~54.94505 ticks per inch
    double TICKINCONVERSION = 54.94505;

    double STRAFEINCONVERSION = 60.000;

    double aprilTagStrafe = 0;

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
        /*
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        //Reversing right side so that runTo is positive
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //Make sure motors are set up at default
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        */
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
                .turn(Math.toRadians(90))
                .forward(-77.5)
                .turn(Math.toRadians(-180))
                .strafeLeft(20.5)
                .addDisplacementMarker(169, () -> {
                    armControl.setPower(1.0);
                    while (armControl.getCurrentPosition() < 1500) {}
                    armControl.setPower(-1.0);
                    //grabberControlLeft.setPosition(0.1);
                    grabberControl.setPower(1.0);
                    while (armControl.getCurrentPosition() > 200) {}
                    armControl.setPower(0.0);
                    grabberControl.setPower(0.0);
                })
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
                .turn(Math.toRadians(90))
                .forward(-79)
                .turn(Math.toRadians(-180))
                .strafeLeft(31.5)
                .addDisplacementMarker(178, () -> {
                    armControl.setPower(1.0);
                    while (armControl.getCurrentPosition() < 1500) {}
                    armControl.setPower(-1.0);
                    //grabberControlLeft.setPosition(0.1);
                    grabberControl.setPower(1.0);
                    while (armControl.getCurrentPosition() > 200) {}
                    armControl.setPower(0.0);
                    grabberControl.setPower(0.0);
                })
                .build();
        TrajectorySequence Middle = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(32.5)
                .forward(-9.3)
                .turn(Math.toRadians(-90))
                .addDisplacementMarker(20, () -> retentionBarControl.setPosition(0.9))
                .addDisplacementMarker(60, () -> retentionBarControl.setPosition(0.5))
                .forward(60)
                .forward(20)// add arm lower in the future
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
                    coneLocation = 2;
                    retentionBarControl.setPosition(0.9);
                    drive.followTrajectorySequence(Middle);
                    armControl.setPower(0.4);
                    while (armControl.getCurrentPosition() < 1000) {}
                    grabberControl.setPower(1.0);
                    long releasePixelStart = System.currentTimeMillis();
                    long releasePixelWaitTime = 1500;//milliseconds
                    long curPixelTime = System.currentTimeMillis();
                    while(curPixelTime-releasePixelStart < releasePixelWaitTime){}
                    grabberControl.setPower(0);
                    armControl.setPower(-0.4);
                    while (armControl.getCurrentPosition() > 250){}
                    armControl.setPower(0.0);
                    grabberControl.setPower(0.0);
                    requestOpModeStop();
                    stage = "putInfrontBoard";
                    break;
                case "rightSpike":
                    coneLocation = 3;
                    drive.followTrajectorySequence(Right);
                    telemetry.addLine("We going right");
                    telemetry.update();
                    stage = "aprilTagInit";
                    break;
                case "leftSpike":
                    coneLocation = 1;
                    telemetry.addLine("We going left");
                    telemetry.update();
                    drive.followTrajectorySequence(Left);
                    stage = "aprilTagInit";
                    break;
                case "putInfrontBoard":
                    requestOpModeStop();
                    stage = "aprilTagInit";
                    break;
                case "aprilTagInit":
                    initAprilTag();
                    stage = "placeOnBoard";
                    break;
                case "placeOnBoard":
                    requestOpModeStop();
                    //Write code to place pixel on board
                    //need a variable cameraOffset which represents the offset of the camera from the grabber
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    telemetry.addData("# of AprilTags Detected", currentDetections.size());
                    for(AprilTagDetection detection : currentDetections) {
                        if(detection.id == coneLocation){
                            targetAprilTag = detection;
                        }
                    }
                    if(targetAprilTag != null){
                        setStrafeTo(targetAprilTag.ftcPose.x);
                    }else{
                        //since we can't tell the x offset without metadata, we are just going to skip this
                    }
                    //Raise the arm to the right height
                    //Distance to bring the arm to the backboard
                    setRunTo(16.0);
                    stage = "park";
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
    //Check the scope that this function should be at
    //Power should be between 0.0 and 1.0
    private void setRunTo(double distance){

        int runTo = (int)(distance * TICKINCONVERSION);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //This is a default value, can be changed as seen fit
        double power = 1.0;

        frontRight.setTargetPosition(runTo);
        frontLeft.setTargetPosition(runTo);
        backRight.setTargetPosition(runTo);
        backLeft.setTargetPosition(runTo);

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);

        telemetry.addData("Power Input", power);
        telemetry.update();

        while (frontRight.isBusy()){}
    }
    private void setRotateTo(double desiredRotation){

        resetWithoutEncoder();
        //I changed this since before, if desiredRotation was 90, then the robot would simply turn until its orientation was 90 instead of turning until its orientation was ADDITIONAL 90.
        double originalAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        //It seems like it needed to be spinning the other way which is why the -1 is there
        double desiredAngle = (originalAngle + (-1*desiredRotation));
        int rotationFactor = 1;

        while(rotationFactor != 0) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double power = 0.5;
            double yawOrientation = orientation.getYaw(AngleUnit.DEGREES);

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();

            if(yawOrientation < (desiredAngle - 1)){
                rotationFactor = 1;
            }else if (yawOrientation > (desiredAngle + 1)){
                rotationFactor = -1;
            } else if ((yawOrientation < (desiredAngle + 1)) && (yawOrientation > (desiredAngle - 1))) {
                rotationFactor = 0;
            }
            frontRight.setPower(power * rotationFactor);
            frontLeft.setPower(-power * rotationFactor);
            backRight.setPower(power * rotationFactor);
            backLeft.setPower(-power * rotationFactor);
        }

    }
    private void setStrafeTo(double distance){
        int runTo = (int)(distance * STRAFEINCONVERSION);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //low because strafing is rough/inaccurate at fast speeds
        double power = 0.4;

        //frontRight & backLeft are negative because strafing causes them to be
        //positive runTo means right strafe
        frontRight.setTargetPosition(-runTo);
        frontLeft.setTargetPosition(runTo);
        backRight.setTargetPosition(runTo);
        backLeft.setTargetPosition(-runTo);

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);

        while(frontRight.isBusy()){}
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
                //.setModelLabels(LABELS)
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


    //Initialize the AprilTag processor
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()

    //Add telemetry about AprilTag detections
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

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}