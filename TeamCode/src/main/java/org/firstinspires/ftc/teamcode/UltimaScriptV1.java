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
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Ultima code Version", group = "Test scripts")
public class UltimaScriptV1 extends LinearOpMode {
    //BIG WHEEl
    BigWheel wheel;
    DcMotor intake;




    // ------------ Odometry constants (tweak for your robot) ------------
    // Encoder spec and odometry wheel geometry
    private static final double TICKS_PER_REV = 8192.0; // encoder CPR (adjust)
    private static final double ODO_WHEEL_RADIUS_IN = 1.378 / 2; // radius in inches (example)
    private static final double TRACK_WIDTH_IN = 10.51; // distance between left and right odometry wheels (inches)
    private static final double LATERAL_WHEEL_OFFSET_IN = 5.25; // perp wheel offset from robot center (inches)
    // -------------------------------------------------------------------

    public static final double feildSize = 144;//in inches,it is square

    public static final double offsetOrginX = feildSize/2;
    public static final double offsetOrginY = feildSize/2;

    public static final double robotSizeY = 14; // in inches
    public static final double robotSizeX = 17.4;



    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime timer = new ElapsedTime();


        // Big wheel testing

        intake = hardwareMap.dcMotor.get("intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wheel = new BigWheel(
                hardwareMap.dcMotor.get("bigWheel"),
                hardwareMap.get(NormalizedColorSensor.class, "colorSens"),
                telemetry
        );

        telemetry.addLine("Welcome to Ultima code Version 1.0, good luck!");
        telemetry.addLine("Pad 1 : drive, intake, pad 2 : mange wheel, shoot");
        telemetry.update();

        boolean aHeld = false;
        boolean bHeld = false;


        // ---------- drive motors (FTCLib) ----------
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

        // ---------- IMU ----------
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu.initialize(imuParams);

        // ---------- Dashboard ----------
        FtcDashboard dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        // ---------- odometry encoders (plugs into motor ports) ----------
        // leftOdo = LF, rightOdo = RF, strafeOdo = LB per your mapping
        DcMotor leftOdo = hardwareMap.get(DcMotor.class, "RF");   // reading only
        DcMotor rightOdo = hardwareMap.get(DcMotor.class, "intake");  // reading only
        DcMotor strafeOdo = hardwareMap.get(DcMotor.class, "LF"); // reading only (perp)

        // Reset and run without encoder (we only read positions)
        leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafeOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Three-wheel odometry helper (units: inches, radians)
        ThreeWheelOdometry odo = new ThreeWheelOdometry(
                TICKS_PER_REV, ODO_WHEEL_RADIUS_IN, TRACK_WIDTH_IN, LATERAL_WHEEL_OFFSET_IN
        );

        int lastLeft = 0;
        int lastRight = 0;
        int lastStrafe = 0;

        odo.reset(
                0,
                0,
                180);

        odo.x = 0 - robotSizeX / 2 ;
        odo.y = 0 - robotSizeY / 2;
        odo.heading = 0;


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // ---------------- drive input ----------------
            driver.readButtons();

            double y = -driver.getLeftY();   // forward/back
            double x = -driver.getLeftX();   // strafe
            double rx = -driver.getRightX(); // rotation

            if (driver.getButton(GamepadKeys.Button.Y)){
                y = y * 0.25;
                x = x * 0.25;
                rx = rx * 0.25;
            }


            if (driver.getButton(GamepadKeys.Button.A)) {
                imu.resetYaw();
            }

            double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // field-centric transform (rotate joystick from field -> robot)
            double rotX = x * Math.cos(-imuHeading) - y * Math.sin(-imuHeading);
            double rotY = x * Math.sin(-imuHeading) + y * Math.cos(-imuHeading);
            rotX *= 1.1; // strafing compensation (tune)

            drive.driveRobotCentric(rotY, rotX, rx);

            // ---------------- odometry update ----------------
            int leftTicks = leftOdo.getCurrentPosition();
            int rightTicks = rightOdo.getCurrentPosition();
            int strafeTicks = strafeOdo.getCurrentPosition();

            int dLeft = leftTicks - lastLeft;
            int dRight = rightTicks - lastRight;
            int dStrafe = strafeTicks - lastStrafe;

            lastLeft = leftTicks;
            lastRight = rightTicks;
            lastStrafe = strafeTicks;

            // update with encoder deltas and imu absolute heading
            odo.update(dLeft, dRight, dStrafe, imuHeading);

            // read pose
            double X = odo.getX();
            double Y = odo.getY();
            double H = odo.getHeading();



            telemetry.addData("frame time", timer.time());
            timer.reset();

            // === Intake System ===
            if (!gamepad2.start) {
                if (gamepad2.a && !aHeld) {
                    boolean accepted = wheel.tryIntake();
                    if (accepted) {
                        // Find what color was just added (previous slot)
                        int lastSlot = (wheel.targetSlot == 0) ? 5 : wheel.targetSlot - 1;
                        String color = wheel.index[lastSlot];
                        telemetry.addLine("Intake: ball accepted (" + color + ")");

                        if (color.equals("P")) {
                            gamepad2.setLedColor(180, 0, 180, 3); // Purple
                        } else if (color.equals("G")) {
                            gamepad2.setLedColor(0, 200, 0, 3);  // Green
                        }
                        gamepad2.rumble(1.0, 1.0, 200);
                    } else {
                        telemetry.addLine("Intake: no ball or slot not open");
                        gamepad2.setLedColor(255, 255, 0, 3); // Yellow fail flash
                    }
                    aHeld = true;
                } else if (!gamepad2.a) {
                    aHeld = false;
                }

                // Intake motor control
                if (gamepad1.x) {
                    intake.setPower(1);
                } else if (gamepad1.back) {
                    intake.setPower(-1);
                } else {
                    intake.setPower(0);
                }

                if (gamepad2.b && !bHeld){
                    wheel.launchBall("P");
                    bHeld = true;
                }
                else{
                    bHeld = false;
                }
            }

            // === Movement Control ===
            wheel.goToCurTarget();

            // === Debug Telemetry ===
            if (gamepad2.y) {
                telemetry.addData("Wheel Pos", wheel.Motor.getCurrentPosition());
                telemetry.addData("Wheel Rev", wheel.Motor.getCurrentPosition() / 537.7);
                telemetry.addData("Target Slot", wheel.targetSlot);
                telemetry.addData("Moving", wheel.isMoving);
                telemetry.addData("error", wheel.PID.getError(wheel.getPos()));

                telemetry.addData("X (in)", "%.2f", X);
                telemetry.addData("Y (in)", "%.2f", Y);
                telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(H));

                telemetry.addData("ODO DF", dLeft);
                telemetry.addData("ODO DR", dRight);
                telemetry.addData("ODO DS", dStrafe);


                for (int i = 0; i < wheel.index.length; i++) {
                    telemetry.addData("Index[" + i + "]", wheel.index[i]);
                }

                telemetry.update();
            }

            // ---------------- dashboard drawing ----------------
            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();
            field.setTranslation(offsetOrginX, offsetOrginY);


            field.setFill("blue");
            field.fillRect(X - robotSizeX / 2.0, Y - robotSizeY / 2.0, robotSizeX, robotSizeY);

            double arrowLen = robotSizeY / 2.0;
            field.setStroke("orange");
            field.strokeLine(X, Y, X + arrowLen * Math.cos(H), Y + arrowLen * Math.sin(H));


            wheel.telemetryColor();

            packet.put("x", X);
            packet.put("y", Y);
            packet.put("headingDeg", Math.toDegrees(H));

            dash.sendTelemetryPacket(packet);
        }

    }

    /**
     * Three-wheel odometry implementation (left, right, strafe).
     * - left/right are forward-parallel encoders (positive forward)
     * - strafe is the perpendicular encoder (positive right)
     * - imuHeading is absolute yaw in radians (continuous)
     *
     * Algorithm notes:
     *  - convert ticks -> inches
     *  - compute dTheta from left/right differential
     *  - correct strafe reading for pure rotation using perp offset
     *  - rotate robot-frame (local) displacement by mid-heading into field frame
     */
    public static class ThreeWheelOdometry {
        private double x = 0;
        private double y = 0;
        private double heading = 0.0; // radians (absolute)

        private final double ticksPerRev;
        private final double wheelRadius;
        private final double trackWidth;    // distance between left and right odos (in)
        private final double perpOffset;    // perp wheel offset from center (in)

        public ThreeWheelOdometry(double ticksPerRev, double wheelRadius, double trackWidth, double perpOffset) {
            this.ticksPerRev = ticksPerRev;
            this.wheelRadius = wheelRadius;
            this.trackWidth = trackWidth;
            this.perpOffset = perpOffset;
        }

        /**
         * Update odometry from encoder deltas (ticks) and absolute IMU heading (radians).
         *
         * @param dLeftTicks  delta left encoder ticks (since last update)
         * @param dRightTicks delta right encoder ticks
         * @param dStrafeTicks delta perpendicular encoder ticks
         * @param imuHeading  absolute IMU heading (radians)
         */
        public void update(int dLeftTicks, int dRightTicks, int dStrafeTicks, double imuHeading) {
            double inchesPerTick = (2.0 * Math.PI * wheelRadius) / ticksPerRev;

            double dLeft = dLeftTicks * inchesPerTick;
            double dRight = dRightTicks * inchesPerTick;
            double dStrafe = dStrafeTicks * inchesPerTick;

            // delta heading from differential of left/right (alternative: use IMU delta, but we use IMU absolute later)
            //double dTheta_enc = (dRight - dLeft) / trackWidth;

            // IMU-based heading change (preferred for absolute)
            double prevHeading = heading;
            double newHeading = imuHeading;
            double dTheta = normalizeAngle(newHeading - prevHeading);

            // correct the perpendicular wheel reading for rotation
            // rotation alone will move the perp wheel by (dTheta * perpOffset)
            double dStrafeCorrected = dStrafe - (dTheta * perpOffset);

            // forward displacement in robot frame
            double dForward = (dLeft + dRight) / 2.0;

            // local robot-frame displacement (forward, right)
            double localX = dForward;
            double localY = dStrafeCorrected;

            // use mid-heading for rotation into field frame
            double headingMid = prevHeading + dTheta * 0.5;

            double fieldDx = localX * Math.cos(headingMid) - localY * Math.sin(headingMid);
            double fieldDy = localX * Math.sin(headingMid) + localY * Math.cos(headingMid);

            x += fieldDx * -1;
            y += fieldDy;
            heading = newHeading;
        }

        private double normalizeAngle(double a) {
            while (a > Math.PI) a -= 2.0 * Math.PI;
            while (a <= -Math.PI) a += 2.0 * Math.PI;
            return a;
        }

        public double getX() { return x; }
        public double getY() { return y; }
        public double getHeading() { return heading; }

        public void reset(double startX, double startY, double startHeading) {
            this.x = startX;
            this.y = startY;
            this.heading = startHeading;
        }
    }
}
