package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.subsystems.constant.TransferConstants.*;

public class TransferSS implements SubsystemInterface {

    private Servo transferServo;
    private Telemetry telemetry;

    private double transferPosition = restPosition;
    private boolean transferActive = false;

    public TransferSS(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public TransferSS(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        transferServo = hardwareMap.get(Servo.class, transferServoName);
    }

    public void read() {

    }

    public void write() {
        transferServo.setPosition(transferPosition);
    }

    public void telemetry() {
        if (telemetry == null) {
            return;
        }

        telemetry.addData("Transfer active", transferActive);
        telemetry.addData("Transfer position", "%.2f", transferPosition);
    }

    public void update() {
        read();
        write();
        telemetry();
    }

    public void activateKicker(boolean isReady) {
        transferActive = isReady;
        transferPosition = isReady ? activePosition : restPosition;
    }

    public void resetKicker() {
        transferActive = false;
        transferPosition = restPosition;
    }

    public boolean isTransferActive() {
        return transferActive;
    }

    public double getTransferPosition() {
        return transferPosition;
    }
}
