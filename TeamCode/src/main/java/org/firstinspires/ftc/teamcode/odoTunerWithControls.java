package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name="⚙ Odo Tuner (MM), with Driving", group="⚙ Tuning")
public class odoTunerWithControls extends LinearOpMode {

    // === DASHBOARD-TUNABLE CONSTANTS (all in MM) ===
    public static double TICKS_PER_REV = 8192.0;         // encoder counts per revolutionz
    public static double WHEEL_RADIUS_MM = 17.5;         // odometry wheel radius in mm
    public static double TRACK_WIDTH_MM = 267.97;         // distance between left & right odometry wheels (mm)
    public static double PERP_OFFSET_MM = 116.078;         // perpendicular wheel offset from center (mm)

    // robot size on dashboard (MM)
    public static double ROBOT_SIZE_X_MM = 441.96;
    public static double ROBOT_SIZE_Y_MM = 355.6;

    // FIELD SIZE (144 in = 3657.6 mm)
    public static double FIELD_SIZE_MM = 3657.6;

    public static boolean IMUTURNING = false; // does the robot use imu to calc turn or uses track offset
    public static boolean controlsOn = true;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        // --- motors ---
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

        // --- odometry class ---
        ThreeWheelOdometry odo = new ThreeWheelOdometry(
                TICKS_PER_REV, WHEEL_RADIUS_MM, TRACK_WIDTH_MM, PERP_OFFSET_MM, IMUTURNING
        );

        int lastL = left.getCurrentPosition();
        int lastR = right.getCurrentPosition();
        int lastS = strafe.getCurrentPosition();

        odo.reset(0.0, 0.0, 0.0); // x(mm), y(mm), heading(rad)

        telemetry.addLine("odoTuner ready. Field units: MM. Press Play.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            int curL = left.getCurrentPosition();
            int curR = right.getCurrentPosition();
            int curS = strafe.getCurrentPosition();

            int dL = curL - lastL;
            int dR = curR - lastR;
            int dS = curS - lastS;

            lastL = curL;
            lastR = curR;
            lastS = curS;

            double headingRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            odo.update(dL, dR, dS, headingRad);

            double X = odo.getX();
            double Y = odo.getY();
            double H = odo.getHeading();

            // telemetry
            telemetry.addData("X (mm)", "%.2f", X);
            telemetry.addData("Y (mm)", "%.2f", Y);
            telemetry.addData("Heading°", "%.2f", Math.toDegrees(H));
            telemetry.addData("dLeft", dL);
            telemetry.addData("dRight", dR);
            telemetry.addData("dStrafe", dS);
            telemetry.update();

            // dashboard field drawing
            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();

            field.setStrokeWidth(2);
            field.setStroke("white");
            //field.strokeRect(-FIELD_SIZE_MM / 2.0, -FIELD_SIZE_MM / 2.0, FIELD_SIZE_MM, FIELD_SIZE_MM);

            field.setFill("blue");
            field.fillRect(
                    (X - ROBOT_SIZE_X_MM / 2.0) / 25.41,// divide by 25.41 to convert to inches for field map
                    (Y - ROBOT_SIZE_Y_MM / 2.0) / 25.41,
                    ROBOT_SIZE_X_MM / 25.41,
                    ROBOT_SIZE_Y_MM / 25.41
            );

            field.setStroke("yellow");
            double arrowLen = Math.max(ROBOT_SIZE_X_MM, ROBOT_SIZE_Y_MM);
            field.strokeLine(
                    X, Y,
                    X + arrowLen * Math.cos(H),
                    Y + arrowLen * Math.sin(H)
            );

            packet.put("X (mm)", X);
            packet.put("Y (mm)", Y);
            packet.put("Heading°", Math.toDegrees(H));

            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            if (gamepad1.a) {
                odo.reset(0, 0, 0);
                odo.x = 0;
                odo.y = 0;
                imu.resetYaw();
            }

            if (controlsOn){
            double cy = -driver.getLeftY();   // forward/back
            double cx = -driver.getLeftX();    // strafe
            double crx = -driver.getRightX();  // rotation

            // Reset yaw if 'A' pressed
            if (driver.getButton(GamepadKeys.Button.A)) {
                imu.resetYaw();
                telemetry.addLine("Yaw reset!");
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Field-centric transform (manual)
            double rotX = cx * Math.cos(-botHeading) - cy * Math.sin(-botHeading);
            double rotY = cx * Math.sin(-botHeading) + cy * Math.cos(-botHeading);

            rotX *= 1.1; // compensate strafing

            // Drive with mecanum kinematics
            drive.driveRobotCentric(rotY, rotX, crx); // use robot-centric here after rotation math
            }
            else{
                drive.driveRobotCentric(0, 0, 0);
            }
        }
    }

    public static class ThreeWheelOdometry {
        private double x = 0.0;
        private double y = 0.0;
        private double heading = 0.0;

        private double encoderHeading = 0.0;   // heading integrated from encoders
        private double lastImuHeading = 0.0;   // heading from previous loop

        private final double ticksPerRev;
        private final double wheelRadius; // mm
        private final double trackWidth;  // mm
        private final double perpOffset;  // mm

        private final boolean imuTurn;

        public ThreeWheelOdometry(double ticksPerRev, double wheelRadius, double trackWidth, double perpOffset, boolean imuTurn) {
            this.ticksPerRev = ticksPerRev;
            this.wheelRadius = wheelRadius;
            this.trackWidth = trackWidth;
            this.perpOffset = perpOffset;
            this.imuTurn = imuTurn;
        }

        public void update(int dLeftTicks, int dRightTicks, int dStrafeTicks, double imuHeading) {

            double mmPerTick = (2.0 * Math.PI * wheelRadius) / ticksPerRev;

            double dLeft = dLeftTicks * mmPerTick;
            double dRight = dRightTicks * mmPerTick;
            double dStrafe = dStrafeTicks * mmPerTick;

            double prevHeading = heading;
            double dTheta;

            if (imuTurn) {
                // use IMU for turning
                dTheta = normalizeAngle(imuHeading - lastImuHeading);
                lastImuHeading = imuHeading;     // save for next cycle
                heading = imuHeading;            // IMU heading IS the real heading
            } else {
                // use encoders for turning
                dTheta = (dRight - dLeft) / trackWidth;
                encoderHeading += dTheta;
                heading = encoderHeading;        // encoder heading IS the real heading
            }

            double dStrafeCorrected = dStrafe - (dTheta * perpOffset);

            // forward from L/R wheels
            double dForward = (dLeft + dRight) / 2.0;

            // convert from local to field coordinates
            double headingMid = prevHeading + (dTheta / 2.0);

            double fieldDx = dForward * Math.cos(headingMid) - dStrafeCorrected * Math.sin(headingMid);
            double fieldDy = dForward * Math.sin(headingMid) + dStrafeCorrected * Math.cos(headingMid);

            x += fieldDx;
            y += fieldDy;
        }

        private double normalizeAngle(double a) {
            while (a > Math.PI) a -= 2.0 * Math.PI;
            while (a <= -Math.PI) a += 2.0 * Math.PI;
            return a;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getHeading() {
            return heading;
        }

        public void reset(double startX, double startY, double startHeading) {
            x = startX;
            y = startY;
            heading = startHeading;
            encoderHeading = startHeading;
            lastImuHeading = startHeading;
        }
    }
}
