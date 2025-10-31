package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class BigWheel {

    private final double kp = 1; // Proportional gain
    private final double ki = 0; // Integral gain
    private final double kd = 0; // Derivative gain

    public PIDController PID = new PIDController(kp,ki,kd);

    public DcMotor Motor;

    public ColorSensor colorSensor;

    public double positionOffset;

    public double[] TargetsPos = {0, 120, 240}; // the targets in degrees
    public double Target = 0; //0, 1 or 2, this is for both intake and launcher
    public double posTolerance = 5;

    public boolean mode = true; // True: intake mode, False: launching mode
    public boolean full = false; // if true no space left in launcher


    double[] purpleColor = {255, 0, 255};
    double[] greenColor = {0, 255, 0};
    final double colorTolerance = 50;

    double[] chamber = {0, 0, 0};
    /**
     * Innit the big wheel
     * @param Motor The DcMotor that runs the wheel
     * @param colorSensor Color sensor in the wheel
     * @param innitPos The starting offset in degrees, (ex: starting in scoring: +30)
     */
    public BigWheel(DcMotor Motor, ColorSensor colorSensor, double innitPos){
        this.Motor = Motor;
        this.colorSensor = colorSensor;
        this.positionOffset = innitPos;
        colorSensor.enableLed(true);
        Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * change the mode of the big wheel
     * @param isIntaking Is the robot in intaking mode, if false robot is shooting
     */
    public void modeSet(boolean isIntaking){
        this.mode = isIntaking;
    }

    public double ballCheck(){
        double r = colorSensor.red();
        double g = colorSensor.green();
        double b = colorSensor.blue();

        double purpleDif = ((r - purpleColor[0]) + (g - purpleColor[1]) + (b - purpleColor[2]));
        double greenDif = ((r - greenColor[0]) + (g - greenColor[1]) + (b - greenColor[2]));
        double output = 0;

        if ((purpleDif > colorTolerance) && (greenDif > colorTolerance)){
            output = 0;
        } else if (purpleDif < colorTolerance) {
            output = 1;
        } else if (greenDif < colorTolerance) {
            output = 2;
        }

        return output;
    }

    public double getPos(){
        return Motor.getCurrentPosition();
    }


    public void goToCurTarget(){
        double curPos = getPos();
        double motorOut = PID.calculateOutput(curPos);
        if (abs(curPos - TargetsPos[(int) (Target)]) <= posTolerance){
            Motor.setPower(motorOut);
        }
        else{
            // if less then tolerance dont do that
            Motor.setPower(0);
        }
    }

    public void main(){
        int index = (int)(Target - 1);
        if (mode && !full){ // INTAKE LOOP
            double currentBall = ballCheck();
            if (currentBall != 0) {
                chamber[index] = currentBall;
                if (chamber[index + 1] != 0) {
                    Target += 1;
                    if (Target > 3) {
                        Target = 1;
                    }
                } else if (chamber[index + 2] != 0) {
                    Target -= 1;
                    if (Target < 1) {
                        Target = 3;
                    }
                } else {
                    full = true;
                }
            }
        }

        // TODO - make it go to the target based on the target it is targeting
        goToCurTarget();
    }



}
