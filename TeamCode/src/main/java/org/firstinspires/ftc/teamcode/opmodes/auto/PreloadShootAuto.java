package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotAuton;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;

public abstract class PreloadShootAuto extends OpMode {
    private RobotAuton robot;
    private final ElapsedTime timer = new ElapsedTime();
    private Step currentStep = Step.WAIT_FOR_SHOOTER;

    protected abstract Alliance alliance();

    private enum Step {
        WAIT_FOR_SHOOTER,
        TRANSFER,
        DONE
    }

    @Override
    public void init() {
        robot = new RobotAuton(hardwareMap, telemetry, alliance());
        robot.startAtAlliancePose();
    }

    @Override
    public void start() {
        currentStep = Step.WAIT_FOR_SHOOTER;
        timer.reset();
    }

    @Override
    public void loop() {
        robot.update();

        switch (currentStep) {
            case WAIT_FOR_SHOOTER:
                if (robot.isShooterReady() || timer.milliseconds() >= RobotConstants.preloadShootTimeoutMs) {
                    robot.transferFor(RobotConstants.preloadShootTransferMs);
                    currentStep = Step.TRANSFER;
                    timer.reset();
                }
                break;

            case TRANSFER:
                if (!robot.isBusy()) {
                    robot.forceIdle();
                    currentStep = Step.DONE;
                }
                break;

            case DONE:
            default:
                robot.forceIdle();
                break;
        }

        telemetry.addData("Preload step", currentStep);
        telemetry.addData("Alliance", alliance());
        telemetry.addData("Shooter ready", robot.isShooterReady());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.forceIdle();
    }
}
