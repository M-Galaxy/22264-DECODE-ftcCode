package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class BigWheel {

    private final double kp = 1; // Proportional gain
    private final double ki = 0; // Integral gain
    private final double kd = 0; // Derivative gain

    public PIDController PID = new PIDController(kp, ki, kd);

    private final double encoderPPR = 537.7;

    int targetSlot = 0;

    // real things
    public DcMotor Motor;
    public NormalizedColorSensor colorSensor;

    public double positionOffset;

    public double Target = 0; // N:3 * PPR this is for both intake and launcher
    public double posTolerance = 5;

    // target colors (normalized)
    final float[] purpleTarget = {0.02f, 0.0f, 0.02f};
    final float[] greenTarget = {0.0f, 0.02f, 0.0f};

    final float[] orangeHoming = {0.5f, 0.5f, 0}; // the orange color used for homing

    final float colorTolerance = 0.5f; // acceptable color difference (0-1 scale)

    // I LOVE IMPORTING TELEMETRY
    Telemetry telemetry;

    String[] index = {"X", "/", "X", "/", "X", "/"}; // G/P : green/purple ball, X : empty slot, / : currently block slot

    public BigWheel(DcMotor Motor, NormalizedColorSensor colorSensor, Telemetry telemetry) {
        this.Motor = Motor;
        this.colorSensor = colorSensor;
        this.positionOffset = (Motor.getCurrentPosition() / encoderPPR);
        Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.telemetry = telemetry;
    }

    // === Utility ===
    public double getPos() {
        return Motor.getCurrentPosition();
    }

    public void goToCurTarget() {
        double curPos = getPos() / encoderPPR;
        double motorOut = PID.calculateOutput(curPos);

        if (abs(curPos - Target) > posTolerance / encoderPPR) {
            Motor.setPower(motorOut);
        } else {
            Motor.setPower(0);
        }
    }

    private double normalizeShortestPath(double current, double target) {
        double diff = target - current;
        // Wrap difference into range (-0.5, 0.5)
        diff -= Math.floor(diff + 0.5);
        return current + diff;
    }

    private void moveToSlot(int slotIndex) {
        double currentRev = getPos() / encoderPPR;
        double rawTarget = slotIndex * (1.0 / 6.0);
        Target = normalizeShortestPath(currentRev, rawTarget);
        PID.setTarget(Target);
        goToCurTarget();
    }

    // === Color Checking ===
    public double ballCheck() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float r = colors.red, g = colors.green, b = colors.blue;

        float purpleDiff = abs(r - purpleTarget[0]) + abs(g - purpleTarget[1]) + abs(b - purpleTarget[2]);
        float greenDiff = abs(r - greenTarget[0]) + abs(g - greenTarget[1]) + abs(b - greenTarget[2]);

        if (purpleDiff < colorTolerance && purpleDiff < greenDiff) return 1;
        else if (greenDiff < colorTolerance && greenDiff < purpleDiff) return 2;
        else return 0;
    }

    public void telemetryColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        telemetry.addData("Raw Red", colors.red);
        telemetry.addData("Raw Green", colors.green);
        telemetry.addData("Raw Blue", colors.blue);

        double detected = ballCheck();
        if (detected == 1) telemetry.addLine("Detected: PURPLE ball");
        else if (detected == 2) telemetry.addLine("Detected: GREEN ball");
        else telemetry.addLine("Detected: None");

        telemetry.update();
    }

    // === SLOT LOGIC ===

    /**
     * Move the specified ball color ("P" or "G") to position 4 and clear it afterward.
     * @param colorCode "P" for purple or "G" for green
     */
    public void launchBall(String colorCode) {
        if (!colorCode.equals("P") && !colorCode.equals("G")) {
            telemetry.addLine("Invalid color input! Use 'P' or 'G'.");
            telemetry.update();
            return;
        }

        int ballPos = -1;

        // Find the ball of the requested color
        for (int i = 0; i < index.length; i++) {
            if (index[i].equals(colorCode)) {
                ballPos = i;
                break;
            }
        }

        if (ballPos == -1) {
            telemetry.addLine("No " + colorCode + " ball found.");
            telemetry.update();
            return;
        }

        // Move to position 4 (launcher position)
        moveToSlot(4);

        // Clear the ball after "launching"
        index[ballPos] = "X";

        telemetry.addLine(colorCode + " ball moved to launch (pos 4) and cleared.");
        telemetry.update();
    }


    /**
     * Wait for a ball input (P or G) and insert into slot 1 if open.
     * Then rotate to next open slot.
     */
    public void intakeBallManual(String inputBall) {
        if (!inputBall.equals("P") && !inputBall.equals("G")) {
            telemetry.addLine("Invalid input (use 'P' or 'G')");
            telemetry.update();
            return;
        }

        int targetSlot = 1;

        if (!index[targetSlot].equals("X")) {
            telemetry.addLine("Slot 1 not open! Waiting...");
            telemetry.update();
            return;
        }

        // Insert new ball
        index[targetSlot] = inputBall;
        telemetry.addData("Added", inputBall + " to slot " + targetSlot);

        // Move to next open slot
        int nextOpen = -1;
        for (int i = 0; i < index.length; i++) {
            if (index[i].equals("X")) {
                nextOpen = i;
                break;
            }
        }

        if (nextOpen != -1) {
            moveToSlot(nextOpen);
            telemetry.addData("Moved to next open slot", nextOpen);
        } else {
            telemetry.addLine("All slots full!");
        }

        telemetry.update();
    }

    /**
     * Automatically waits for a ball detected by the color sensor.
     * When a ball (purple or green) is detected, it is added to slot 1 if open,
     * then moves to the next available open slot.
     */
    public void intakeBall() {
        telemetry.addLine("Waiting for ball detection...");
        telemetry.update();

        // Wait until a ball is detected by color sensor
        double detected = 0;
        while (detected == 0) {
            detected = ballCheck(); // 0 = none, 1 = purple, 2 = green
        }

        String inputBall = (detected == 1) ? "P" : "G";
        telemetry.addData("Detected Ball", inputBall);
        telemetry.update();



        if (!index[targetSlot].equals("X")) {
            telemetry.addLine("Slot 1 not open! Waiting...");
            telemetry.update();
            return;
        }

        // Insert new ball into slot 1
        index[targetSlot] = inputBall;
        telemetry.addData("Added", inputBall + " to slot " + targetSlot);

        // Move to next open slot
        int nextOpen = -1;
        for (int i = 0; i < index.length; i++) {
            if (index[i].equals("X")) {
                nextOpen = i + 1; //I HCINGAOBSAJDB
                break;
            }
        }

        if (nextOpen != -1) {
            moveToSlot(nextOpen);
            telemetry.addData("Moved to next open slot", nextOpen);
        } else {
            telemetry.addLine("All slots full!");
        }

        telemetry.update();
    }
}
