package org.firstinspires.ftc.teamcode.drive.opmode;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.openftc.easyopencv.OpenCvCamera;



import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.easyopencv.OpenCvCamera;

@TeleOp
public class DriverControl2023 extends LinearOpMode {

    //initialize motors
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;


    private DcMotor armControl;

    private Servo grabberControl;
    private Servo launcherControl;

    //private TouchSensor touchSensor;

    //init sensors/camera
    OpenCvCamera camera;



    @Override
    public void runOpMode() {

        //init motors
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");



        armControl = hardwareMap.get(DcMotor.class, "armControl");
        grabberControl = hardwareMap.get(Servo.class, "grabberControl");
        launcherControl = hardwareMap.get(Servo.class, "launcherControl");

        //touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;

        double grabberControlPosition = grabberControl.getPosition();

        int baseArmHeight = -1*armControl.getCurrentPosition();

        //double check which motors are reversed, assumption is right-side
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        armControl.setDirection(DcMotorSimple.Direction.REVERSE);
   /*
   frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
   backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
   frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
   backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

   frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

   telemetry.addData("Starting at",  "%7d :%7d",
           frontLeft.getCurrentPosition(),
           backLeft.getCurrentPosition(),
           frontRight.getCurrentPosition(),
           backRight.getCurrentPosition());

    */


        telemetry.addLine("Init Done");

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");




            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            double up = 0;
            double down = 0;
            double manualControl = 0;
            double strength = 0.5;
            up = gamepad2.right_trigger;
            if(armControl.getCurrentPosition() >= baseArmHeight) {
                down = gamepad2.left_trigger;
            }
           /*if(gamepad2.dpad_down){
               manualControl = -0.2;
           }
           if(gamepad2.dpad_up) {
               manualControl = 0.2;
           }*/
            double armPower = (up - down + manualControl);
            armControl.setPower(armPower);









            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;

            if(gamepad1.dpad_left){
                frontLeftPower += 0.3;
                backLeftPower += 0.3;
                frontRightPower += -0.3;
                backRightPower += -0.3;
            }
            if(gamepad1.dpad_right){
                frontLeftPower += -0.3;
                backLeftPower += -0.3;
                frontRightPower += 0.3;
                backRightPower += 0.3;
            }
            if(gamepad1.dpad_up){
                frontLeftPower += -0.3;
                backLeftPower += -0.3;
                frontRightPower += -0.3;
                backRightPower += -0.3;
            }
            if(gamepad1.dpad_down){
                frontLeftPower += 0.3;
                backLeftPower += 0.3;
                frontRightPower += 0.3;
                backRightPower += 0.3;
            }
            if(gamepad1.right_bumper){
                strength = 1.0;
            }
            if(gamepad1.left_bumper){
                strength = 0.25;
            }




            /*if(gamepad2.x && grabberControl.getPosition() != 1){
                grabberControlPosition
                grabberControl.setPosition(.8);
            }
            if(gamepad2.a && grabberControl.getPosition() != 0){
                grabberControl.setPosition(0.1);
            }
            if(gamepad2.dpad_up && grabberControl.getPosition() > 0.1){
                grabberControlPosition += 0.1
                grabberControl.setPosition(grabberControlPosition);
            }
            if(gamepad2.a && grabberControl.getPosition() != 0){
                grabberControl.setPosition(0.1);
            }*/

            if(gamepad2.x && grabberControl.getPosition() != 1){
                grabberControlPosition = 0.8;
            }
            if(gamepad2.a && grabberControl.getPosition() != 0){
                grabberControlPosition = 0.55;
            }
            /*if(gamepad2.dpad_up && grabberControl.getPosition() <= 0.9){
                grabberControlPosition += 0.01;
            }
            if(gamepad2.dpad_down && grabberControl.getPosition() >= 0.1){
                grabberControlPosition -= 0.01;
            }*/

            grabberControl.setPosition(grabberControlPosition);


            if(gamepad2.dpad_up && launcherControl.getPosition() != 1){
                launcherControl.setPosition(.60);
            }
            if(gamepad2.dpad_down && launcherControl.getPosition() != 0){
                launcherControl.setPosition(0.1);
            }

            /*if (touchSensor.isPressed()) {
                telemetry.addLine("Cone Aligned = True");
            }*/

            //*0.5 to set more reasonable speed
            frontRight.setPower(frontRightPower*strength);
            frontLeft.setPower(frontLeftPower*strength);
            backRight.setPower(backRightPower*strength);
            backLeft.setPower(backLeftPower*strength);


            //last line
            telemetry.addData("Front Right Position",
                    //frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition());
            //backRight.getCurrentPosition());


            //telemetry.addData("Touch Sensor Pressed: ", touchSensor.isPressed());
            telemetry.addData("Front Right Speed",
                    frontRight.getPower());
            telemetry.addData("Front Left Speed",
                    frontLeft.getPower());
            telemetry.addData("Back Right Speed",
                    backRight.getPower());
            telemetry.addData("Back Left Speed",
                    backLeft.getPower());
            if (strength == 0.5) {
                telemetry.addLine("Power?");
            } else {
                telemetry.addLine("Power!");
            }
            telemetry.addData("Arm Control Speed",
                    armControl.getPower());
            telemetry.addData("ArmControlBaseHeight",
                    baseArmHeight);
            telemetry.addData("ArmControlPosition",
                    armControl.getCurrentPosition());
            telemetry.addData("GrabberPosition",
                    grabberControl.getPosition());
            telemetry.addLine("updated");


            telemetry.update();
        }
    }
}


