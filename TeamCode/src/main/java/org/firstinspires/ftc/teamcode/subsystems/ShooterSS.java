package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.PIDFSController;
import org.firstinspires.ftc.teamcode.math.ShooterEquation;

import static org.firstinspires.ftc.teamcode.subsystems.constant.ShooterConstants.shooterMotorName;
import static org.firstinspires.ftc.teamcode.subsystems.constant.ShooterConstants.velocityToleranceRPM;

public class ShooterSS implements SubsystemInterface {

    private ShooterEquation shooterEquation;
    private PIDFSController pidfsController;
    private DcMotorEx shooterMotor;
    private Telemetry telemetry;

    private double shooterPow = 0.0;
    private double currentVel;
    private double targetRPM = 0.0;

    public ShooterSS(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public ShooterSS(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        shooterEquation = new ShooterEquation();
        pidfsController = new PIDFSController(hardwareMap);
        shooterMotor = hardwareMap.get(DcMotorEx.class, shooterMotorName);
    }

    public void read() {
        currentVel = shooterMotor.getVelocity() * 60.0 / 28.0;
    }

    public void write() {
        shooterMotor.setPower(shooterPow);
    }

    public void telemetry() {
        if (telemetry == null) {
            return;
        }

        telemetry.addData("Shooter target RPM", "%.1f", targetRPM);
        telemetry.addData("Shooter current RPM", "%.1f", currentVel);
        telemetry.addData("Shooter error RPM", "%.1f", targetRPM - currentVel);
        telemetry.addData("Shooter power", "%.3f", shooterPow);
        telemetry.addData("Shooter ready", isReady());
    }

    public void update() {
        read();
        write();
        telemetry();
    }

    public void spinShooter(double robotX, double robotY, double goalX, double goalY) {
        spinShooter(robotX, robotY, goalX, goalY, 0.0);
    }

    public void spinShooter(double robotX, double robotY, double goalX, double goalY, double targetRPMOffset) {
        double distance = Math.hypot(goalX - robotX, goalY - robotY);

        targetRPM = shooterEquation.getTargetRPM(distance) + targetRPMOffset;

        shooterPow = pidfsController.calculate(targetRPM, currentVel);
    }

    public void spinShooterRPM(double targetRPM) {
        this.targetRPM = targetRPM;
        shooterPow = pidfsController.calculate(targetRPM, currentVel);
    }

    public void stopShooter() {
        targetRPM = 0.0;
        shooterPow = 0.0;
        pidfsController.reset();
    }

    public boolean isReady() {
        return targetRPM > 0.0 && Math.abs(targetRPM - currentVel) <= velocityToleranceRPM;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentVelRPM() {
        return currentVel;
    }

    public double getShooterPow() {
        return shooterPow;
    }
}
