package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.OdometrySubsystem;

/**
 * TwoWheelOdometry estimates robot position (x, y, heading) based on
 * two wheel encoders and IMU heading. No hardware dependencies.
 */
public class TwoWheelOdo {

    private double x; // X position in inches
    private double y; // Y position in inches
    private double heading; // Radians

    //TODO - make have real measurements

    private final double wheelBase = 12.0;       // inches   // Distance between left and right wheels (inches)
    private final double ticksPerRev = 8192;  // Encoder ticks per wheel revolution
    private final double wheelRadius = 1;  // Wheel radius in inches

    public TwoWheelOdo(double xStart, final double yStart) {
        this.x = xStart;
        this.y = yStart;
        this.heading = 0;
    }

    /**
     * Update position based on encoder deltas and heading.
     *
     * @param leftDeltaTicks  Change in left encoder ticks since last update
     * @param rightDeltaTicks Change in right encoder ticks since last update
     * @param imuHeading      Current IMU heading (radians)
     */
    public void update(double leftDeltaTicks, double rightDeltaTicks, double imuHeading) {
        // Convert ticks to distance
        double leftDistance = 2 * Math.PI * wheelRadius * (leftDeltaTicks / ticksPerRev);
        double rightDistance = 2 * Math.PI * wheelRadius * (rightDeltaTicks / ticksPerRev);

        double deltaHeading = imuHeading - heading; // change in heading since last update
        heading = imuHeading;

        // Approximate movement
        double forward = (leftDistance + rightDistance) / 2.0;
        double dx = forward * Math.cos(heading);
        double dy = forward * Math.sin(heading);

        x += dx;
        y += dy;
    }

    /** Getters for position */
    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }

    /** Reset odometry */
    public void reset(double startX, double startY, double startHeading) {
        this.x = startX;
        this.y = startY;
        this.heading = startHeading;
    }
}
