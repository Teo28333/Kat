package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.TransferSS;
import org.firstinspires.ftc.teamcode.subsystems.TurretSS;
import org.firstinspires.ftc.teamcode.subsystems.constant.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.constant.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.constant.TransferConstants;
import org.firstinspires.ftc.teamcode.subsystems.constant.TurretConstants;

import java.util.Locale;

public final class PanelsDebug {
    private static final double ROBOT_RADIUS = 9.0;

    private static final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private static final Style robotStyle = new Style("", "#3F51B5", 0.75);
    private static final Style pathStyle = new Style("", "#FF9800", 0.75);
    private static final Style historyStyle = new Style("", "#4CAF50", 0.75);

    private static boolean initialized = false;

    private PanelsDebug() {
    }

    public static void init() {
        if (initialized) {
            return;
        }

        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        PanelsConfigurables.INSTANCE.refreshClass(new ShooterConstants());
        PanelsConfigurables.INSTANCE.refreshClass(new TurretConstants());
        PanelsConfigurables.INSTANCE.refreshClass(new IntakeConstants());
        PanelsConfigurables.INSTANCE.refreshClass(new TransferConstants());
        PanelsConfigurables.INSTANCE.refreshClass(new RobotConstants());

        initialized = true;
    }

    public static void update(Follower follower, ShooterSS shooter, TurretSS turret,
                              IntakeSS intake, TransferSS transfer, String commandState) {
        init();

        Pose pose = follower.getPose();
        Vector velocity = follower.getVelocity();

        panelsTelemetry.addData("Pose X", round(pose.getX()));
        panelsTelemetry.addData("Pose Y", round(pose.getY()));
        panelsTelemetry.addData("Pose Heading Deg", round(Math.toDegrees(pose.getHeading())));
        if (velocity != null) {
            panelsTelemetry.addData("Velocity X", round(velocity.getXComponent()));
            panelsTelemetry.addData("Velocity Y", round(velocity.getYComponent()));
        }

        panelsTelemetry.addData("Command State", commandState);
        panelsTelemetry.addData("Transfer Active", transfer.isTransferActive());
        panelsTelemetry.addData("Transfer Position", round(transfer.getTransferPosition()));

        panelsTelemetry.addData("Shooter Target RPM", round(shooter.getTargetRPM()));
        panelsTelemetry.addData("Shooter Current RPM", round(shooter.getCurrentVelRPM()));
        panelsTelemetry.addData("Shooter Error RPM", round(shooter.getTargetRPM() - shooter.getCurrentVelRPM()));
        panelsTelemetry.addData("Shooter Power", round(shooter.getShooterPow()));
        panelsTelemetry.addData("Shooter Ready", shooter.isReady());

        panelsTelemetry.addData("Turret Target Deg", round(turret.getTargetAngleDeg()));
        panelsTelemetry.addData("Turret Current Deg", round(turret.getCurrentAngleDeg()));
        panelsTelemetry.addData("Turret Power", round(turret.getTurretPow()));
        panelsTelemetry.addData("Turret At Target", turret.isAtTarget());

        panelsTelemetry.addData("Front Roller Power", round(intake.getFrontRollerPow()));
        panelsTelemetry.addData("Conveyor Power", round(intake.getConveyorPow()));
        panelsTelemetry.addData("Front Roller Velocity", round(intake.getFrontRollerCurrentVel()));
        panelsTelemetry.addData("Conveyor Velocity", round(intake.getConveyorCurrentVel()));

        drawFollower(follower);
        panelsTelemetry.update();
    }

    private static void drawFollower(Follower follower) {
        Path currentPath = follower.getCurrentPath();
        if (currentPath != null) {
            drawPath(currentPath, pathStyle);
        }

        PathChain currentPathChain = follower.getCurrentPathChain();
        if (currentPathChain != null) {
            drawPath(currentPathChain, pathStyle);
        }

        PoseHistory poseHistory = follower.getPoseHistory();
        if (poseHistory != null) {
            drawPoseHistory(poseHistory, historyStyle);
        }

        drawRobot(follower.getPose(), robotStyle);
        panelsField.update();
    }

    private static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector heading = pose.getHeadingAsUnitVector();
        heading.setMagnitude(ROBOT_RADIUS);
        double x1 = pose.getX() + heading.getXComponent() / 2.0;
        double y1 = pose.getY() + heading.getYComponent() / 2.0;
        double x2 = pose.getX() + heading.getXComponent();
        double y2 = pose.getY() + heading.getYComponent();

        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    private static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();
        if (points == null || points.length < 2 || points[0].length == 0) {
            return;
        }

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0.0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    private static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    private static void drawPoseHistory(PoseHistory poseHistory, Style style) {
        panelsField.setStyle(style);

        int size = poseHistory.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {
            panelsField.moveCursor(poseHistory.getXPositionsArray()[i], poseHistory.getYPositionsArray()[i]);
            panelsField.line(poseHistory.getXPositionsArray()[i + 1], poseHistory.getYPositionsArray()[i + 1]);
        }
    }

    private static String round(double value) {
        return String.format(Locale.US, "%.2f", value);
    }
}
