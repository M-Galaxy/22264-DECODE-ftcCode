package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@TeleOp(name="Big Wheel Test", group="Test scripts")
public class BigWheelTest extends LinearOpMode {

    BigWheel wheel;

    @Override
    public void runOpMode() throws InterruptedException {

        wheel = new BigWheel(
                hardwareMap.dcMotor.get("bigWheel"),
                hardwareMap.get(NormalizedColorSensor.class, "colorSens"),
                telemetry
        );

        telemetry.addLine("Initialized BigWheel Test. Use gamepad2 controls.");
        telemetry.addLine("A = Intake Mode | B = Launch Mode | Y = Telemetry");
        telemetry.update();

        boolean aHeld = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (!gamepad2.start){
                //for intake
                if (gamepad2.a && !aHeld){
                    wheel.intakeBall();
                    aHeld = true;
                }
                else if (!gamepad1.a){
                    aHeld = false;
                }
            }else{
                if (gamepad2.x){
                    wheel.launchBall("P");
                } else if (gamepad2.a) {
                    wheel.launchBall("G");
                }
            }

            wheel.goToCurTarget();

            if (gamepad2.y) {
                telemetry.addData("Wheel Position", wheel.Motor.getCurrentPosition());
                telemetry.addData("Wheel / Resolution", wheel.Motor.getCurrentPosition() / 537.7);

                telemetry.addData("target", wheel.targetSlot);

                String[] indexs = wheel.index;

                for (int i = 0; i < indexs.length; i++) {
                    String title = "index postion, " + i;
                    telemetry.addData(title, indexs[i]);
                }


                wheel.telemetryColor();


                telemetry.update();
            }
        }
    }
}
