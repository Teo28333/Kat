package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;

public final class PoseStorage {
    public static Pose currentPose = null;

    private PoseStorage() {
    }

    public static boolean hasPose() {
        return currentPose != null && isValid(currentPose);
    }

    public static Pose getOrAllianceStart(Alliance alliance) {
        return hasPose() ? currentPose : alliance.startPose();
    }

    public static void setCurrentPose(Pose pose) {
        if (isValid(pose)) {
            currentPose = pose;
        }
    }

    public static void clear() {
        currentPose = null;
    }

    public static boolean isValid(Pose pose) {
        return pose != null
                && !Double.isNaN(pose.getX())
                && !Double.isNaN(pose.getY())
                && !Double.isNaN(pose.getHeading());
    }
}
