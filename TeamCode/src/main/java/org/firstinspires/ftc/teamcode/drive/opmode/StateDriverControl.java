package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.openftc.easyopencv.OpenCvCamera;

@TeleOp(group = "drive")
public class StateDriverControl extends LinearOpMode {

    //initialize motors
    private DcMotor frontRight, frontLeft, backRight, backLeft;

    private DcMotor armControl;
    private Servo retentionBarControl;

    private CRServo grabberControl;

    private DcMotor hangingLeft, hangingRight;

    //private Servo hangReleaseLeft, hangReleaseRight;

    //private TouchSensor touchSensor;

    //init sensors/camera
    OpenCvCamera camera;

    private double retentionBarPosition;

    private boolean hanging = false;

    @Override
    public void runOpMode() {

        //init driveTrain motors
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        armControl = hardwareMap.get(DcMotor.class, "armControl");
        retentionBarControl = hardwareMap.get(Servo.class, "retentionBarControl");
        grabberControl = hardwareMap.get(CRServo.class, "grabberControl");

        hangingLeft = hardwareMap.get(DcMotor.class, "hangingLeft");
        hangingRight = hardwareMap.get(DcMotor.class, "hangingRight");


        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;

        retentionBarPosition = retentionBarControl.getPosition();

        int baseArmHeight = armControl.getCurrentPosition();

        //double check which motors are reversed, assumption is right-side

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);


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
            if(gamepad2.dpad_down){
                if(armControl.getCurrentPosition() < baseArmHeight){
                    baseArmHeight = armControl.getCurrentPosition();
                }
                manualControl = -0.5;
            }
            if(gamepad2.dpad_up) {
                manualControl = 0.5;
            }
            double armPower = (up - down + manualControl);
            if(!hanging){
                armControl.setPower(armPower*0.5);
            }



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


            //Code openRetentionBar(boolean) using moveToPosition *ONLY ON RETENTION BAR MOTOR*
            if(gamepad2.left_bumper){
                retentionBarPosition = 0.5;
            }
            if(gamepad2.right_bumper){
                //openRetentionBar(false);
                retentionBarPosition = 0.9;
            }


            retentionBarControl.setPosition(retentionBarPosition);


            double grabberControlPower = 0;
            if(gamepad2.right_stick_button){
                //ReleasePixel
                grabberControlPower = 0.75;
            }
            if(gamepad2.left_stick_button){
                //GrabPixel
                grabberControlPower = -0.75;
            }

            grabberControl.setPower(grabberControlPower);

            if(gamepad2.y){
                hanging = true;
            }
            if(gamepad2.a){
                hanging = false;
            }
            double hangingControlPower = 0;
            hangingControlPower = gamepad2.left_trigger - gamepad2.right_trigger;

            if(hanging){
                hangingLeft.setPower(hangingControlPower);
                hangingRight.setPower(hangingControlPower);
            }



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

            telemetry.addData("RetentionBarPosition",
                    retentionBarControl.getPosition());
            telemetry.addLine("updated");


            telemetry.update();

        }
    }
}


