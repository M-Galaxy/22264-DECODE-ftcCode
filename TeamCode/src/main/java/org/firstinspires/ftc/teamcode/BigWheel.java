package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class BigWheel {

    private final double kp = 1; // Proportional gain
    private final double ki = 0; // Integral gain
    private final double kd = 0; // Derivative gain

    private final double encoderPPR = 537.7;

    public PIDController PID = new PIDController(kp,ki,kd);

    public DcMotor Motor;

    public NormalizedColorSensor colorSensor;

    public double positionOffset;

    public double LauncherModeOffset = 60;
    public double Target = 0; //N:3 * PPR this is for both intake and launcher
    public double posTolerance = 5;

    public boolean mode = true; // True: intake mode, False: launching mode
    public boolean full = false; // if true no space left in launcher
    public boolean empty = true; // to tell if there is no balls in intake

    // target colors (normalized)
    final float[] purpleTarget = {0.75f, 0.0f, 0.75f};  // purple
    final float[] greenTarget = {0.0f, 0.75f, 0.0f};   // green
    final float colorTolerance = 0.5f; // acceptable color difference (0-1 scale)

    double[] chamber = {0, 0, 0};
    /**
     * Innit the big wheel
     * @param Motor The DcMotor that runs the wheel
     //* @param colorSensor Color sensor in the wheel
     */
    public BigWheel(DcMotor Motor, NormalizedColorSensor  colorSensor){
        this.Motor = Motor;
        this.colorSensor = colorSensor;
        this.positionOffset = (Motor.getCurrentPosition()/encoderPPR);
        //colorSensor.enableLed(true);
        Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * change the mode of the big wheel
     * @param isIntaking Is the robot in intaking mode, if false robot is shooting
     */
    public void modeSet(boolean isIntaking){
        this.mode = isIntaking;
    }

    /**
     * Detect which color ball is currently seen by the color sensor.
     * @return 0 = none, 1 = purple, 2 = green
     */
    public double ballCheck() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Normalize RGB to 0–1 range
        float r = colors.red;
        float g = colors.green;
        float b = colors.blue;

        // Compute simple color differences
        float purpleDiff = abs(r - purpleTarget[0]) + abs(g - purpleTarget[1]) + abs(b - purpleTarget[2]);
        float greenDiff = abs(r - greenTarget[0]) + abs(g - greenTarget[1]) + abs(b - greenTarget[2]);

        double output = 0;
        if (purpleDiff < colorTolerance && purpleDiff < greenDiff) {
            output = 1; // purple
        } else if (greenDiff < colorTolerance && greenDiff < purpleDiff) {
            output = 2; // green
        }


        return output;
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


    public double getPos(){
        return Motor.getCurrentPosition();
    }


    public void goToCurTarget() {
        double curPos = getPos() / encoderPPR; // convert ticks → revs
        double motorOut = PID.calculateOutput(curPos);

        if (abs(curPos - Target) > posTolerance / encoderPPR) {
            Motor.setPower(motorOut);
        } else {
            Motor.setPower(0);
        }
    }


//    public boolean launchColor (double ballNeeded){
//        if (chamber)
//    }


public void intakeLoop() {
    if (mode && !full) { // Only run intake logic if in intake mode and not full
        double currentBall = ballCheck();

        // If a ball is detected (non-zero color)
        if (currentBall != 0) {
            // Find the next open slot
            int openIndex = -1;
            for (int i = 0; i < chamber.length; i++) {
                if (chamber[i] == 0) {
                    openIndex = i;
                    break;
                }
            }

            // If there’s an open slot, store the ball and rotate to the next slot
            if (openIndex != -1) {
                chamber[openIndex] = currentBall;
                Target = (openIndex + 1) * (1.0 / 3.0); // 1/3 rev per slot
                PID.setTarget(Target);
            } else {
                // All slots full
                full = true;
            }
        }
    }
    // Move wheel to current target position
    goToCurTarget();


}

public void emptyWheelLoop() {
    if (!mode && !empty) { // false = launching mode, check if there are any balls left
        boolean hasBall = false;
        for (double slot : chamber) {
            if (slot != 0) {
                hasBall = true;
                break;
            }
        }

        if (hasBall) {
            // Rotate 1/3 revolution to reach the next chamber slot
            // + 1/6 revolution offset to align with output/launcher
            Target += (1.0 / 3.0) + (1.0 / 6.0);

            // Wrap around after 1 full rotation
            if (Target >= 1.0) {
                Target -= 1.0;
            }

            PID.setTarget(Target);

            // "launch the front-most ball
            for (int i = 0; i < chamber.length; i++) {
                if (chamber[i] != 0) {
                    chamber[i] = 0; // slot emptied
                    break; // only empty one per cycle
                }
            }
        } else {
            empty = true; // wheel is now empty
        }
    }

    // Move motor toward current target
    goToCurTarget();
}





}
