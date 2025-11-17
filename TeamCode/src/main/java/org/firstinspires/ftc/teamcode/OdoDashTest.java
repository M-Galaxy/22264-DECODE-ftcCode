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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

// Dashboard
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;

// Custom Odometry
import org.firstinspires.ftc.teamcode.TwoWheelOdo;

@TeleOp(name = "Field Centric + Odo Dashboard", group = "Main")
public class OdoDashTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //  MOTOR SETUP
        Motor frontLeft = new Motor(hardwareMap, "LF");
        Motor backLeft = new Motor(hardwareMap, "LB");
        Motor frontRight = new Motor(hardwareMap, "RF");
        Motor backRight = new Motor(hardwareMap, "RB");

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        MecanumDrive drive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

        GamepadEx driver = new GamepadEx(gamepad1);

        //  IMU SETUP
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu.initialize(params);


        //  DASHBOARD
        FtcDashboard dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        // ODO INIT
        TwoWheelOdo odo = new TwoWheelOdo(0, 0);

        //  ENCODERS FOR ODOMETRY
        Motor parallelEnc = backLeft;
        Motor perpEnc = frontLeft;

        parallelEnc.resetEncoder();
        perpEnc.resetEncoder();

        int lastParallel = 0;
        int lastPerp = 0;

        waitForStart();
        if (isStopRequested()) return;


        //  MAIN LOOP
        while (opModeIsActive()) {

            driver.readButtons();


            //  FIELD CENTRIC DRIVE MATH
            double y = -driver.getLeftY();
            double x = -driver.getLeftX();
            double rx = -driver.getRightX();

            if (driver.getButton(GamepadKeys.Button.A)) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate joystick into field frame
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1; // strafing compensation

            drive.driveRobotCentric(rotY, rotX, rx);


            //  UPDATE ODOMETRY
            int parallelPos = parallelEnc.getCurrentPosition();
            int perpPos = perpEnc.getCurrentPosition();

            int dPar = parallelPos - lastParallel;
            int dPerp = perpPos - lastPerp;

            lastParallel = parallelPos;
            lastPerp = perpPos;

            odo.update(dPar, dPerp, botHeading);

            double X = odo.getX();
            double Y = odo.getY();
            double H = odo.getHeading();

            //  DASHBOARD DRAWING
            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();

            double robotSize = 9;
            field.setFill("blue");
            field.fillRect(X - robotSize / 2, Y - robotSize / 2, robotSize, robotSize);

            // Heading arrow
            double L = 12;
            field.setStroke("orange");
            field.strokeLine(
                    X,
                    Y,
                    X + L * Math.cos(H),
                    Y + L * Math.sin(H)
            );

            packet.put("x", X);
            packet.put("y", Y);
            packet.put("headingDeg", Math.toDegrees(H));

            dash.sendTelemetryPacket(packet);
        }
    }
}
