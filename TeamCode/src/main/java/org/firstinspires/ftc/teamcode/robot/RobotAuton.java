package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemInterface;
import org.firstinspires.ftc.teamcode.subsystems.TransferSS;
import org.firstinspires.ftc.teamcode.subsystems.TurretSS;

import static org.firstinspires.ftc.teamcode.robot.RobotConstants.*;

public class RobotAuton {

    public final Follower follower;
    public final IntakeSS intake;
    public final ShooterSS shooter;
    public final TurretSS turret;
    public final TransferSS transfer;
    public final IntakeCommands intakeCommands;
    private final SubsystemInterface[] subsystems;

    private final Telemetry telemetry;
    private final Alliance alliance;
    private final ElapsedTime actionTimer = new ElapsedTime();

    private State currentState = State.IDLE;
    private double actionTimeoutMs = 0.0;
    private double pathIntakeTimeoutMs = -1.0;

    private enum State {
        IDLE,
        INTAKING,
        TRANSFERRING,
        FOLLOWING,
        FOLLOWING_AND_INTAKE
    }

    public RobotAuton(HardwareMap hardwareMap, Telemetry telemetry, boolean isBlueAlliance) {
        this(hardwareMap, telemetry, isBlueAlliance ? Alliance.BLUE : Alliance.RED);
    }

    public RobotAuton(HardwareMap hardwareMap, Telemetry telemetry, Alliance alliance) {
        this.telemetry = telemetry;
        this.alliance = alliance;

        follower = Constants.createFollower(hardwareMap);
        intake = new IntakeSS(hardwareMap, telemetry);
        shooter = new ShooterSS(hardwareMap, telemetry);
        turret = new TurretSS(hardwareMap, telemetry);
        transfer = new TransferSS(hardwareMap, telemetry);
        intakeCommands = new IntakeCommands(intake, transfer);
        subsystems = new SubsystemInterface[]{intake, shooter, turret, transfer};
        PanelsDebug.init();
    }

    public void start(Pose startingPose) {
        if (startingPose == null) {
            throw new IllegalArgumentException("RobotAuton startingPose cannot be null");
        }

        follower.setStartingPose(startingPose);
    }

    public void startAtAlliancePose() {
        start(alliance.startPose());
    }

    public void update() {
        follower.update();

        Pose pose = follower.getPose();
        double robotX = pose.getX();
        double robotY = pose.getY();
        double heading = pose.getHeading();

        shooter.spinShooter(robotX, robotY, alliance.shootingGoalX(), alliance.shootingGoalY());
        turret.aimAtTarget(robotX, robotY, heading, alliance.aimGoalX(), alliance.aimGoalY());

        updateState();
        intakeCommands.update(shooter.isReady());

        updateAllSubsystems();
        Pose currentPose = follower.getPose();
        PoseStorage.setCurrentPose(currentPose);

        telemetry.addData("Auton state", currentState);
        telemetry.addData("Shooter ready", shooter.isReady());
        telemetry.addData("Path progress", "%.1f", getPathProgressPercent());
        addRobotTelemetry(currentPose);
        PanelsDebug.update(follower, shooter, turret, intake, transfer, intakeCommands.getState());
    }

    public void intakeFor(double timeoutMs) {
        intakeCommands.intake();
        actionTimeoutMs = timeoutMs;
        currentState = State.INTAKING;
        actionTimer.reset();
    }

    public void transferFor(double timeoutMs) {
        intakeCommands.transfer();
        actionTimeoutMs = timeoutMs;
        currentState = State.TRANSFERRING;
        actionTimer.reset();
    }

    public void followPath(Path path) {
        followPath(path, true);
    }

    public void followPath(Path path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        currentState = State.FOLLOWING;
    }

    public void followPath(PathChain path) {
        followPath(path, true);
    }

    public void followPath(PathChain path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        currentState = State.FOLLOWING;
    }

    public void followPathAndIntake(Path path) {
        followPathAndIntake(path, true);
    }

    public void followPathAndIntake(Path path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        intakeCommands.intake();
        pathIntakeTimeoutMs = -1.0;
        currentState = State.FOLLOWING_AND_INTAKE;
    }

    public void followPathAndIntake(PathChain path) {
        followPathAndIntake(path, true);
    }

    public void followPathAndIntake(PathChain path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        intakeCommands.intake();
        pathIntakeTimeoutMs = -1.0;
        currentState = State.FOLLOWING_AND_INTAKE;
    }

    public void followPathAndIntakeFor(Path path, double intakeTimeoutMs) {
        followPathAndIntakeFor(path, intakeTimeoutMs, true);
    }

    public void followPathAndIntakeFor(Path path, double intakeTimeoutMs, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        intakeCommands.intake();
        pathIntakeTimeoutMs = intakeTimeoutMs;
        currentState = State.FOLLOWING_AND_INTAKE;
        actionTimer.reset();
    }

    public void followPathAndIntakeFor(PathChain path, double intakeTimeoutMs) {
        followPathAndIntakeFor(path, intakeTimeoutMs, true);
    }

    public void followPathAndIntakeFor(PathChain path, double intakeTimeoutMs, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        intakeCommands.intake();
        pathIntakeTimeoutMs = intakeTimeoutMs;
        currentState = State.FOLLOWING_AND_INTAKE;
        actionTimer.reset();
    }

    public boolean isBusy() {
        return currentState != State.IDLE;
    }

    public boolean isShooterReady() {
        return shooter.isReady();
    }

    public boolean isBlueAlliance() {
        return alliance.isBlue();
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void forceIdle() {
        follower.breakFollowing();
        intakeCommands.idle();
        shooter.stopShooter();
        turret.stopTurret();
        transfer.resetKicker();
        currentState = State.IDLE;
        pathIntakeTimeoutMs = -1.0;
        updateAllSubsystems();
    }

    public double getPathProgressPercent() {
        if (!follower.isBusy()) {
            return 100.0;
        }

        return Math.max(0.0, Math.min(100.0, follower.getPathCompletion() * 100.0));
    }

    public void updateAllSubsystems() {
        for (SubsystemInterface subsystem : subsystems) {
            subsystem.update();
        }
    }

    private void updateState() {
        switch (currentState) {
            case INTAKING:
                if (actionTimer.milliseconds() >= actionTimeoutMs) {
                    intakeCommands.idle();
                    currentState = State.IDLE;
                }
                break;

            case TRANSFERRING:
                if (actionTimer.milliseconds() >= actionTimeoutMs) {
                    intakeCommands.idle();
                    currentState = State.IDLE;
                } else {
                    intakeCommands.transfer();
                }
                break;

            case FOLLOWING:
                if (!follower.isBusy()) {
                    currentState = State.IDLE;
                }
                break;

            case FOLLOWING_AND_INTAKE:
                if (pathIntakeTimeoutMs >= 0.0 && actionTimer.milliseconds() >= pathIntakeTimeoutMs) {
                    intakeCommands.idle();
                    pathIntakeTimeoutMs = -1.0;
                }

                if (!follower.isBusy()) {
                    intakeCommands.idle();
                    pathIntakeTimeoutMs = -1.0;
                    currentState = State.IDLE;
                }
                break;

            case IDLE:
            default:
                break;
        }
    }

    private void addRobotTelemetry(Pose pose) {
        telemetry.addData("Robot pose", "%.1f, %.1f, %.1f deg",
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading()));
        telemetry.addData("Target shooter RPM", "%.1f", shooter.getTargetRPM());
        telemetry.addData("Current shooter RPM", "%.1f", shooter.getCurrentVelRPM());
        telemetry.addData("Target turret angle deg", "%.1f", turret.getTargetAngleDeg());
        telemetry.addData("Current turret angle deg", "%.1f", turret.getCurrentAngleDeg());
    }

}
