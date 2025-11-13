package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Demo & Game (RUN ME)", group="Non-game scripts")
public class DemoAndGame extends LinearOpMode {
    final boolean DEBUG = true;

    final double speedReduct = 0.2;

    final double PPRLauncher = 28;

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

        boolean game = false;


        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor leftLauncher = hardwareMap.dcMotor.get("outL");
        DcMotor rightLauncher = hardwareMap.dcMotor.get("outR");

        leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        ElapsedTime timer = new ElapsedTime();

        double lastPosL = 0;
        double lastPosR = 0;

        while (opModeIsActive()) {
            // all of my keybinds stored nice and pretty

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            boolean opt = gamepad1.options;

            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean bx = gamepad1.x; //to dictate button x not x as in the stick


            boolean bump = gamepad1.left_bumper;

            if (!game) {

                // DEMO DRIVE SETTINGS -----------------------


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

                frontLeftMotor.setPower(frontLeftPower*speedReduct);
                backLeftMotor.setPower(backLeftPower*speedReduct);
                frontRightMotor.setPower(frontRightPower*speedReduct);
                backRightMotor.setPower(backRightPower*speedReduct);

                if (bump) {
                    leftLauncher.setPower(-0.22);
                    rightLauncher.setPower(0.22);
                    telemetry.addData("Shooting", true);
                } else {
                    leftLauncher.setPower(0);
                    rightLauncher.setPower(0);
                }

                if (a){
                    intake.setPower(0.75);
                    gamepad1.setLedColor(0,255,0,100);
                    telemetry.addData("Intakeing", true);
                } else if (b) {
                    intake.setPower(-0.75);
                    gamepad1.setLedColor(255,0,0,100);
                    telemetry.addData("Outakeing", true);
                } else{
                    gamepad1.setLedColor(255,255,255,100);
                    intake.setPower(0);
                }
            }
            else { // GAME SETTING ------------------------
                if (bump){
                    intake.setPower(-0.75);
                    telemetry.addData("Intake on", true);
                }
                else{
                    intake.setPower(0);
                }

                double launchPower = 0;

                if (b){
                    launchPower = 0.5;
                    telemetry.addData("Level 1 launch", true);
                    gamepad1.setLedColor(255,0,0,200);
                } else if (a) {
                    launchPower = 0.475;
                    telemetry.addData("Level 2 launch", true);
                    gamepad1.setLedColor(255,255,0,200);
                }else if (bx){
                    launchPower = 0.45;
                    telemetry.addData("Level 3 launch", true);
                    gamepad1.setLedColor(0,255,0,200);
                }
                else if (gamepad1.y){
                    launchPower = 0.425;
                    telemetry.addData("Level 3 launch", true);
                    gamepad1.setLedColor(0,255,0,200);
                }

                if (gamepad2.dpad_left){
                    launchPower = -0.1;
                    telemetry.addData("BREAK", true);
                    gamepad1.setLedColor(255,255,255,200);
                }

                leftLauncher.setPower(-launchPower);
                rightLauncher.setPower(launchPower);

                double deltaTime = timer.seconds();

                double deltaTicksL = (leftLauncher.getCurrentPosition() - lastPosL);
                double deltaTicksR = (rightLauncher.getCurrentPosition() - lastPosR);

                double tpsL = deltaTicksL / deltaTime;
                double tpsR = deltaTicksR / deltaTime;

                telemetry.addData("Left launcher RMP", (tpsL/PPRLauncher));
                telemetry.addData("Right launcher RMP", (tpsR/PPRLauncher));


                telemetry.addData("Avg RMP", (((tpsR/PPRLauncher)+(tpsL/PPRLauncher))/2));

                lastPosL = leftLauncher.getCurrentPosition();
                lastPosR = rightLauncher.getCurrentPosition();
            }


            // Regardless of setting keybinds
            if (gamepad1.dpad_up){
                game = true;
            } else if (gamepad1.dpad_down) {
                game = false;
            }


            timer.reset();
            telemetry.update();
        }
    }
}