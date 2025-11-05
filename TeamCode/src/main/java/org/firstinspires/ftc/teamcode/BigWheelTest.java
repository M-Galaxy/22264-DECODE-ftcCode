package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@TeleOp(name="Big Wheel Test")
public class BigWheelTest extends LinearOpMode {

    BigWheel wheel;

    @Override
    public void runOpMode() throws InterruptedException {

        wheel = new BigWheel(
                hardwareMap.dcMotor.get("bigWheel"),
                hardwareMap.get(NormalizedColorSensor.class, "colorSens")
        );

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Control mode switching
            if (gamepad1.a) wheel.modeSet(true);   // Intake mode
            if (gamepad1.b) wheel.modeSet(false);  // Launch mode

            // Run the correct loop based on mode
            if (wheel.mode) {
                wheel.intakeLoop();
            } else {
                wheel.emptyWheelLoop();
            }

            // Telemetry outputs
            if (gamepad1.y) {
                telemetry.addData("Wheel Position", wheel.Motor.getCurrentPosition());
                telemetry.addData("Wheel / Resolution", wheel.Motor.getCurrentPosition() / 537.7);
                telemetry.addData("Mode", wheel.mode ? "Intake" : "Launch");
                telemetry.addData("Full", wheel.full);
                telemetry.addData("Empty", wheel.empty);

                // Add color telemetry
                wheel.telemetryColor();
            }
            telemetry.update();
        }
    }
}
