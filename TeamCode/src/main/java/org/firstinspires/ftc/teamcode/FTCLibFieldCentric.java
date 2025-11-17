package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;


//ftc dash
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

// Odo
import com.arcrobotics.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="FTCLib Field Centric", group="Test scripts")
public class FTCLibFieldCentric extends LinearOpMode {

    final boolean DEBUG = true;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Motors ---
        Motor frontLeft = new Motor(hardwareMap, "LF");
        Motor backLeft = new Motor(hardwareMap, "LB");
        Motor frontRight = new Motor(hardwareMap, "RF");
        Motor backRight = new Motor(hardwareMap, "RB");

        // Brake to prevent drift
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // --- Mecanum drive ---
        MecanumDrive drive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

        // --- Gamepad ---
        GamepadEx driver = new GamepadEx(gamepad1);

        // Retrieve the IMU from the hardware map
        com.qualcomm.robotcore.hardware.IMU imu = hardwareMap.get(com.qualcomm.robotcore.hardware.IMU.class, "imu");
        com.qualcomm.robotcore.hardware.IMU.Parameters parameters = new com.qualcomm.robotcore.hardware.IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        // Initialize dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        // Send telemetry both to DS and dashboard
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            driver.readButtons();

            double y = -driver.getLeftY();   // forward/back
            double x = -driver.getLeftX();    // strafe
            double rx = -driver.getRightX();  // rotation

            // Reset yaw if 'A' pressed
            if (driver.getButton(GamepadKeys.Button.A)) {
                imu.resetYaw();
                telemetry.addLine("Yaw reset!");
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Field-centric transform (manual)
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1; // compensate strafing

            // Drive with mecanum kinematics
            drive.driveRobotCentric(rotY, rotX, rx); // use robot-centric here after rotation math



            if (DEBUG) {
                telemetry.addData("Front Left Power", frontLeft.get());
                telemetry.addData("Back Left Power", backLeft.get());
                telemetry.addData("Front Right Power", frontRight.get());
                telemetry.addData("Back Right Power", backRight.get());
                telemetry.addData("Heading (deg)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            }

            telemetry.update();
        }
    }
}
