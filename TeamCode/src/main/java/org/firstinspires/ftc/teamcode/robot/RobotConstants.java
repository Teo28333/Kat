package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class RobotConstants {
    public static double driveSpeedMultiplier = 1.0;
    public static double turnSpeedMultiplier = 0.75;

    public static double startXRed = 8.25;
    public static double startYRed = 8.5;
    public static double startHeadingRed = 0.0;

    public static double startXBlue = 135.75;
    public static double startYBlue = 8.5;
    public static double startHeadingBlue = Math.PI;

    public static double aimGoalXRed = 132.0;
    public static double aimGoalYRed = 132.0;
    public static double aimGoalXBlue = 12.0;
    public static double aimGoalYBlue = 132.0;

    public static double shootingGoalXRed = 144.0;
    public static double shootingGoalYRed = 144.0;
    public static double shootingGoalXBlue = 0.0;
    public static double shootingGoalYBlue = 144.0;

    public static double preloadShootTransferMs = 1500.0;
    public static double preloadShootTimeoutMs = 4000.0;

    public static Pose redStartPose() {
        return new Pose(startXRed, startYRed, startHeadingRed);
    }

    public static Pose blueStartPose() {
        return new Pose(startXBlue, startYBlue, startHeadingBlue);
    }
}
