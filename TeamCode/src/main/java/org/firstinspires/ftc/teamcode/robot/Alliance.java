package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;

public enum Alliance {
    RED,
    BLUE;

    public boolean isBlue() {
        return this == BLUE;
    }

    public Pose startPose() {
        return isBlue() ? RobotConstants.blueStartPose() : RobotConstants.redStartPose();
    }

    public double aimGoalX() {
        return isBlue() ? RobotConstants.aimGoalXBlue : RobotConstants.aimGoalXRed;
    }

    public double aimGoalY() {
        return isBlue() ? RobotConstants.aimGoalYBlue : RobotConstants.aimGoalYRed;
    }

    public double shootingGoalX() {
        return isBlue() ? RobotConstants.shootingGoalXBlue : RobotConstants.shootingGoalXRed;
    }

    public double shootingGoalY() {
        return isBlue() ? RobotConstants.shootingGoalYBlue : RobotConstants.shootingGoalYRed;
    }
}
