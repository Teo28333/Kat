package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
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

public class Robot {

    private final Telemetry telemetry;
    private final Alliance alliance;

    private final Follower follower;
    private final IntakeSS intake;
    private final ShooterSS shooter;
    private final TurretSS turret;
    private final TransferSS transfer;
    private final IntakeCommands intakeCommands;
    private final SubsystemInterface[] subsystems;
    private final ElapsedTime teleOpTimer = new ElapsedTime();

    private boolean lastIntakeToggle = false;
    private boolean endgameRumbled = false;
    private boolean lastTurretOffsetLeft = false;
    private boolean lastTurretOffsetRight = false;
    private boolean lastShooterOffsetUp = false;
    private boolean lastShooterOffsetDown = false;

    private double turretTargetOffsetDeg = 0.0;
    private double shooterTargetOffsetRPM = 0.0;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean isBlueAlliance) {
        this(hardwareMap, telemetry, isBlueAlliance ? Alliance.BLUE : Alliance.RED);
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Alliance alliance) {
        this.telemetry = telemetry;
        this.alliance = alliance;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.getOrAllianceStart(alliance));

        intake = new IntakeSS(hardwareMap, telemetry);
        shooter = new ShooterSS(hardwareMap, telemetry);
        turret = new TurretSS(hardwareMap, telemetry);
        transfer = new TransferSS(hardwareMap, telemetry);
        intakeCommands = new IntakeCommands(intake, transfer);
        subsystems = new SubsystemInterface[]{intake, shooter, turret, transfer};

        PanelsDebug.init();
    }

    public void start() {
        teleOpTimer.reset();
        endgameRumbled = false;
        follower.startTeleopDrive();
    }

    public void update(Gamepad gamepad1) {
        update(gamepad1, null);
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        Pose pose = follower.getPose();
        double robotX = pose.getX();
        double robotY = pose.getY();
        double heading = pose.getHeading();

        updateTargetOffsets(gamepad2);
        drive(gamepad1);
        shooter.spinShooter(robotX, robotY, alliance.shootingGoalX(), alliance.shootingGoalY(), shooterTargetOffsetRPM);
        turret.aimAtTarget(robotX, robotY, heading, alliance.aimGoalX(), alliance.aimGoalY(), turretTargetOffsetDeg);
        runIntakeCommands(gamepad1);
        rumbleAfterOneMinuteForty(gamepad1);

        follower.update();
        updateAllSubsystems();
        Pose currentPose = follower.getPose();
        PoseStorage.setCurrentPose(currentPose);

        telemetry.addData("Alliance", alliance);
        telemetry.addData("TeleOp time", "%.1f", teleOpTimer.seconds());
        telemetry.addData("Shooter RPM offset", "%.1f", shooterTargetOffsetRPM);
        telemetry.addData("Turret angle offset", "%.1f", turretTargetOffsetDeg);
        telemetry.addData("Intake", intakeCommands.getState());
        telemetry.addData("Shooter ready", shooter.isReady());
        addRobotTelemetry(currentPose);
        telemetry.update();

        PanelsDebug.update(follower, shooter, turret, intake, transfer, intakeCommands.getState());
    }

    public Follower getFollower() {
        return follower;
    }

    public boolean isBlueAlliance() {
        return alliance.isBlue();
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void updateAllSubsystems() {
        for (SubsystemInterface subsystem : subsystems) {
            subsystem.update();
        }
    }

    private void drive(Gamepad gamepad1) {
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * driveSpeedMultiplier,
                -gamepad1.left_stick_x * driveSpeedMultiplier,
                -gamepad1.right_stick_x * turnSpeedMultiplier,
                true
        );
    }

    private void updateTargetOffsets(Gamepad gamepad2) {
        if (gamepad2 == null) {
            return;
        }

        if (gamepad2.dpad_left && !lastTurretOffsetLeft) {
            turretTargetOffsetDeg -= 5.0;
        }
        if (gamepad2.dpad_right && !lastTurretOffsetRight) {
            turretTargetOffsetDeg += 5.0;
        }
        if (gamepad2.dpad_up && !lastShooterOffsetUp) {
            shooterTargetOffsetRPM += 25.0;
        }
        if (gamepad2.dpad_down && !lastShooterOffsetDown) {
            shooterTargetOffsetRPM -= 25.0;
        }

        lastTurretOffsetLeft = gamepad2.dpad_left;
        lastTurretOffsetRight = gamepad2.dpad_right;
        lastShooterOffsetUp = gamepad2.dpad_up;
        lastShooterOffsetDown = gamepad2.dpad_down;
    }

    private void runIntakeCommands(Gamepad gamepad1) {
        boolean intakePressed = gamepad1.right_bumper && !lastIntakeToggle;
        if (intakePressed) {
            if (intakeCommands.isIntaking()) {
                intakeCommands.idle();
            } else {
                intakeCommands.intake();
            }
        }

        if (gamepad1.b) {
            intakeCommands.outtake();
        } else if (intakeCommands.isOuttaking()) {
            intakeCommands.idle();
        }

        if (gamepad1.left_bumper) {
            intakeCommands.transfer();
        } else if (intakeCommands.isTransferring()) {
            intakeCommands.idle();
        }

        intakeCommands.update(shooter.isReady());
        lastIntakeToggle = gamepad1.right_bumper;
    }

    private void rumbleAfterOneMinuteForty(Gamepad gamepad1) {
        if (!endgameRumbled && teleOpTimer.seconds() >= 100.0) {
            gamepad1.rumble(1.0, 1.0, 750);
            endgameRumbled = true;
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
