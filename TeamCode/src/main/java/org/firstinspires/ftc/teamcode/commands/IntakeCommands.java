package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.TransferSS;

import static org.firstinspires.ftc.teamcode.subsystems.constant.TransferConstants.*;

public class IntakeCommands {

    private final IntakeSS intake;
    private final TransferSS transfer;
    private final ElapsedTime transferTimer = new ElapsedTime();

    private Mode currentMode = Mode.IDLE;
    private TransferPhase transferPhase = TransferPhase.FEEDING;

    private enum Mode {
        IDLE,
        INTAKING,
        OUTTAKING,
        TRANSFERRING
    }

    private enum TransferPhase {
        FEEDING,
        KICKING,
        WAITING
    }

    public IntakeCommands(IntakeSS intake, TransferSS transfer) {
        this.intake = intake;
        this.transfer = transfer;
    }

    public void update(boolean shooterReady) {
        switch (currentMode) {
            case INTAKING:
                transfer.resetKicker();
                intake.intake();
                break;

            case OUTTAKING:
                transfer.resetKicker();
                intake.outtake();
                break;

            case TRANSFERRING:
                updateTransfer(shooterReady);
                break;

            case IDLE:
            default:
                transfer.resetKicker();
                intake.stopIntake();
                break;
        }
    }

    public void intake() {
        setMode(Mode.INTAKING);
    }

    public void outtake() {
        setMode(Mode.OUTTAKING);
    }

    public void transfer() {
        setMode(Mode.TRANSFERRING);
    }

    public void idle() {
        setMode(Mode.IDLE);
    }

    public boolean isIntaking() {
        return currentMode == Mode.INTAKING;
    }

    public boolean isOuttaking() {
        return currentMode == Mode.OUTTAKING;
    }

    public boolean isTransferring() {
        return currentMode == Mode.TRANSFERRING;
    }

    public String getState() {
        if (currentMode == Mode.TRANSFERRING) {
            return currentMode + " / " + transferPhase;
        }
        return currentMode.toString();
    }

    private void setMode(Mode mode) {
        if (currentMode == mode) {
            return;
        }

        currentMode = mode;
        if (mode == Mode.TRANSFERRING) {
            transferPhase = TransferPhase.FEEDING;
            transferTimer.reset();
        } else {
            transfer.resetKicker();
        }
    }

    private void updateTransfer(boolean shooterReady) {
        switch (transferPhase) {
            case FEEDING:
                intake.intake();
                transfer.resetKicker();
                if (transferTimer.seconds() >= intakeBeforeKickSec) {
                    transferPhase = TransferPhase.KICKING;
                    transferTimer.reset();
                }
                break;

            case KICKING:
                intake.stopIntake();
                transfer.activateKicker(true);
                if (transferTimer.seconds() >= kickerActiveSec) {
                    transferPhase = TransferPhase.WAITING;
                    transferTimer.reset();
                }
                break;

            case WAITING:
            default:
                intake.stopIntake();
                transfer.resetKicker();
                if (transferTimer.seconds() >= waitAfterKickSec) {
                    transferPhase = TransferPhase.FEEDING;
                    transferTimer.reset();
                }
                break;
        }
    }
}
