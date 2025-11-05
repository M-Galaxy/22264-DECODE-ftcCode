package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Full and AutoWheel Robot Test")
public class FullTestCodeWithWheel extends LinearOpMode {

    final boolean DEBUG = true;

    private final double encoderPPR = 537.7;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("LF");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("LB");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("RF");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("RB");

        double target = 0;
        double targetFin = 0;
        PIDController manipPID = new PIDController(0.0075, 0, 0); //0.005); // Tune these!

        DcMotor manipulator = hardwareMap.dcMotor.get("bigWheel");
        manipulator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        manipulator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        manipulator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean lastBumpL = false;
        boolean lastBumpR = false;


        // Reverse the right side motors.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //STOP THE DRIFT
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);



        DcMotor leftLauncher = hardwareMap.dcMotor.get("outL");
        DcMotor rightLauncher = hardwareMap.dcMotor.get("outR");

        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE); // make both spin same way

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, I have no idea why every stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            boolean opt = gamepad1.options;

            if (DEBUG) {
                telemetry.addData("Y drive", y);
                telemetry.addData("X drive", x);
                telemetry.addData("RX drive", rx);
            }

            if (opt) {
                imu.resetYaw();
                telemetry.addData("Yaw reset", rx);
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (DEBUG) {telemetry.addData("Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));}

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            if (DEBUG) {
                telemetry.addData("rOTATED X", rotX);
                telemetry.addData("rOTATED Y", rotY);
            }

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (DEBUG) {
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Right Power", backRightPower);
            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            double launchStrength = gamepad2.right_trigger;  // range: 0.0 - 1.0

            leftLauncher.setPower(-launchStrength);
            rightLauncher.setPower(-launchStrength);

            if (launchStrength > 0.05) {
                telemetry.addData("Shooting", true);
                telemetry.addData("Launch Strength", launchStrength);
            } else {
                telemetry.addData("Shooting", false);
            }



            boolean a = gamepad1.a;
            boolean b = gamepad1.b;

            DcMotor intake = hardwareMap.dcMotor.get("intake");
            if (a){
                intake.setPower(1);
                gamepad1.setLedColor(0,255,0,100);
                telemetry.addData("Intakeing", true);
            } else if (b) {
                intake.setPower(-1);
                gamepad1.setLedColor(255,0,0,100);
                telemetry.addData("Outakeing", true);
            } else{
                gamepad1.setLedColor(255,255,255,100);
                intake.setPower(0);
            }

            //manip part
            boolean bumpL = gamepad2.left_bumper;
            boolean bumpR = gamepad2.right_bumper;
            boolean shift = gamepad2.a;

            if (bumpL && !lastBumpL) {
                target += (1.0 / 3.0) * encoderPPR;
            }
            if (bumpR && !lastBumpR) {
                target -= (1.0 / 3.0) * encoderPPR;
            }
            if (shift) {
                targetFin = target + ((1.0 / 3.0) * encoderPPR);
            }

            lastBumpL = bumpL;
            lastBumpR = bumpR;

            manipPID.setTarget(target);
            double manipPower = manipPID.calculateOutput(manipulator.getCurrentPosition());
            manipulator.setPower(manipPower / 2);

            telemetry.addData("Manipulator Target", target);
            telemetry.addData("Manipulator Pos", manipulator.getCurrentPosition());
            telemetry.addData("Manipulator Power", manipPower);


            telemetry.update();
        }
    }
}