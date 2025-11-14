package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@TeleOp(name="Big Wheel Test", group="Test scripts")
public class BigWheelTest extends LinearOpMode {

    BigWheel wheel;
    DcMotor intake;

    @Override
    public void runOpMode() throws InterruptedException {

        intake = hardwareMap.dcMotor.get("intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wheel = new BigWheel(
                hardwareMap.dcMotor.get("bigWheel"),
                hardwareMap.get(NormalizedColorSensor.class, "colorSens"),
                telemetry
        );

        telemetry.addLine("Initialized BigWheel Test. Use gamepad2 controls.");
        telemetry.addLine("A = Intake Ball | X/Back = Intake Motor | Y = Telemetry");
        telemetry.update();

        boolean aHeld = false;
        boolean bHeld = false;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

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
                    telemetry.update();
                    aHeld = true;
                } else if (!gamepad2.a) {
                    aHeld = false;
                }

                // Intake motor control
                if (gamepad2.x) {
                    intake.setPower(1);
                } else if (gamepad2.back) {
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

                for (int i = 0; i < wheel.index.length; i++) {
                    telemetry.addData("Index[" + i + "]", wheel.index[i]);
                }

                wheel.telemetryColor();
                telemetry.update();
            }
        }
    }
}
