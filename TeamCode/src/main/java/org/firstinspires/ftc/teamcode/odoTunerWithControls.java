package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;


@Config
@TeleOp(name="⚙ Odo Tuner Pedro Mecanum", group="⚙ Tuning")
public class OdoTunerPedroMecanum extends LinearOpMode {



    public static double TICKS_PER_REV = 8192.0;
    public static double WHEEL_RADIUS_MM = 17.5;
    public static double TRACK_WIDTH_MM = 267.97;
    public static double PERP_OFFSET_MM = 116.078;

    public static double ROBOT_SIZE_X_MM = 441.96;
    public static double ROBOT_SIZE_Y_MM = 355.6;

    public static double FIELD_SIZE_MM = 3657.6;

    public static boolean IMUTURNING = false;
    public static boolean controlsOn = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // --- Drive motors ---
        Motor frontLeft = new Motor(hardwareMap, "LF");
        Motor backLeft = new Motor(hardwareMap, "LB");
        Motor frontRight = new Motor(hardwareMap, "RF");
        Motor backRight = new Motor(hardwareMap, "RB");

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // --- Pedro Pathing Mecanum drive ---
        MecanumDrive drive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

        GamepadEx driver = new GamepadEx(gamepad1);

        // --- Odometry motors ---
        DcMotor left = hardwareMap.dcMotor.get("RF");
        DcMotor right = hardwareMap.dcMotor.get("intake");
        DcMotor strafe = hardwareMap.dcMotor.get("LF");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- IMU ---
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        // --- Pedro Pathing localizer ---
        ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
                .forwardTicksToMm((2 * Math.PI * WHEEL_RADIUS_MM) / TICKS_PER_REV)
                .strafeTicksToMm((2 * Math.PI * WHEEL_RADIUS_MM) / TICKS_PER_REV)
                .turnTicksToMm((2 * Math.PI * WHEEL_RADIUS_MM) / TICKS_PER_REV)
                .leftPodY(-TRACK_WIDTH_MM / 2.0)
                .rightPodY(TRACK_WIDTH_MM / 2.0)
                .strafePodX(PERP_OFFSET_MM)
                .leftEncoderHardwareMapName("RF")
                .rightEncoderHardwareMapName("intake")
                .strafeEncoderHardwareMapName("LF");

        ThreeWheelLocalizer odo;
        if (IMUTURNING) {
            odo = new ThreeWheelIMULocalizer(hardwareMap, imu, localizerConstants);
        } else {
            odo = new ThreeWheelLocalizer(hardwareMap, localizerConstants);
        }

        odo.setPose(new Pose2d(0,0,0));

        telemetry.addLine("Pedro OdoTuner with Mecanum ready. Field units: MM. Press Play.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            odo.update();
            Pose2d pose = odo.getPose();

            double X = pose.getX();
            double Y = pose.getY();
            double H = pose.getHeading();

            // telemetry
            telemetry.addData("X (mm)", "%.2f", X);
            telemetry.addData("Y (mm)", "%.2f", Y);
            telemetry.addData("Heading°", "%.2f", Math.toDegrees(H));
            telemetry.update();

            // --- Dashboard drawing ---
            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();
            field.setStrokeWidth(2);
            field.setStroke("white");
            field.setFill("blue");
            field.fillRect(
                    (X - ROBOT_SIZE_X_MM/2)/25.41,
                    (Y - ROBOT_SIZE_Y_MM/2)/25.41,
                    ROBOT_SIZE_X_MM/25.41,
                    ROBOT_SIZE_Y_MM/25.41
            );

            field.setStroke("yellow");
            double arrowLen = Math.max(ROBOT_SIZE_X_MM, ROBOT_SIZE_Y_MM);
            field.strokeLine(
                    X/25.41, Y/25.41,
                    (X + arrowLen * Math.cos(H))/25.41,
                    (Y + arrowLen * Math.sin(H))/25.41
            );

            packet.put("X (mm)", X);
            packet.put("Y (mm)", Y);
            packet.put("Heading°", Math.toDegrees(H));
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            // --- Controls using Pedro Mecanum ---
            if (controlsOn) {
                double forward = -driver.getLeftY();
                double strafe = -driver.getLeftX();
                double rotate = -driver.getRightX();

                if (driver.getButton(GamepadKeys.Button.A)) {
                    odo.setPose(new Pose2d(0,0,0));
                    imu.resetYaw();
                    telemetry.addLine("Yaw reset!");
                    telemetry.update();
                }

                // Pedro Mecanum: field-centric drive
                drive.driveFieldCentric(forward, strafe, rotate, H);
            } else {
                drive.driveRobotCentric(0,0,0);
            }
        }
    }
}
