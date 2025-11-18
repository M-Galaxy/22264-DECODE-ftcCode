package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class positions {

    public static final Pose2d START_LEFT =
            new Pose2d(10, 60, new Rotation2d(Math.toRadians(90)));

    public static final Pose2d START_RIGHT =
            new Pose2d(40, 60, new Rotation2d(Math.toRadians(90)));

    public static final Pose2d CYCLE_STACK =
            new Pose2d(75, 35, new Rotation2d(Math.toRadians(180)));

    public static final Pose2d CHAMBER_SCORE =
            new Pose2d(110, 50, new Rotation2d(Math.toRadians(0)));

    public static final Pose2d PARK =
            new Pose2d(120, 10, new Rotation2d(Math.toRadians(0)));
}
