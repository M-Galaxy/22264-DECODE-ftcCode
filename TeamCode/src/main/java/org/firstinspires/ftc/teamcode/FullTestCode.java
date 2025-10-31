package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Full Robot Test")
public class FullTestCode extends LinearOpMode {

    final boolean DEBUG = true;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("LF");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("LB");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("RF");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("RB");

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

            boolean bump = gamepad1.left_bumper;

            DcMotor leftLauncher = hardwareMap.dcMotor.get("outL");
            DcMotor rightLauncher = hardwareMap.dcMotor.get("outR");
            if (bump){
                leftLauncher.setPower(-1);
                rightLauncher.setPower(1);
                telemetry.addData("Shooting", true);
            }
            else{
                leftLauncher.setPower(0);
                rightLauncher.setPower(0);
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


            double manipulatorSpin = (gamepad1.left_trigger - gamepad1.right_trigger) / 4;

            DcMotor manipulator = hardwareMap.dcMotor.get("bigWheel");

            telemetry.addData("Bigwheel spin", manipulatorSpin);
            manipulator.setPower(manipulatorSpin);

            telemetry.update();
        }
    }
}