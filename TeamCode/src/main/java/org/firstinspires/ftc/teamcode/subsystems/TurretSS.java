package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.subsystems.constant.TurretConstants.*;

public class TurretSS implements SubsystemInterface {

    private DcMotorEx turretMotor;
    private Telemetry telemetry;

    private double turretPow = 0.0;
    private double currentAngleDeg = 0.0;
    private double targetAngleDeg = 0.0;
    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastTimeNs = -1L;

    public TurretSS(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public TurretSS(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        turretMotor = hardwareMap.get(DcMotorEx.class, turretMotorName);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void read() {
        currentAngleDeg = encoderTicksToDegrees(turretMotor.getCurrentPosition());
    }

    public void write() {
        turretMotor.setPower(turretPow);
    }

    public void telemetry() {
        if (telemetry == null) {
            return;
        }

        telemetry.addData("Turret target deg", "%.1f", targetAngleDeg);
        telemetry.addData("Turret current deg", "%.1f", currentAngleDeg);
        telemetry.addData("Turret error deg", "%.1f", normalizeDegrees(targetAngleDeg - currentAngleDeg));
        telemetry.addData("Turret power", "%.3f", turretPow);
        telemetry.addData("Turret at target", isAtTarget());
    }

    public void update() {
        read();
        write();
        telemetry();
    }

    public void aimAtTarget(double robotX, double robotY, double heading, double goalX, double goalY) {
        aimAtTarget(robotX, robotY, heading, goalX, goalY, 0.0);
    }

    public void aimAtTarget(double robotX, double robotY, double heading, double goalX, double goalY, double targetAngleOffsetDeg) {
        double fieldAngleDeg = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));
        targetAngleDeg = clampToTurretRange(normalizeDegrees(fieldAngleDeg - Math.toDegrees(heading) + targetAngleOffsetDeg));

        turretPow = calculateTurretPower(targetAngleDeg, currentAngleDeg);
    }

    public void stopTurret() {
        turretPow = 0.0;
        resetController();
    }

    public boolean isAtTarget() {
        return Math.abs(normalizeDegrees(targetAngleDeg - currentAngleDeg)) <= angleToleranceDeg;
    }

    public double getCurrentAngleDeg() {
        return currentAngleDeg;
    }

    public double getTargetAngleDeg() {
        return targetAngleDeg;
    }

    public double getTurretPow() {
        return turretPow;
    }

    public void aimToAngleDeg(double targetAngleDeg) {
        this.targetAngleDeg = clampToTurretRange(normalizeDegrees(targetAngleDeg));
        turretPow = calculateTurretPower(this.targetAngleDeg, currentAngleDeg);
    }

    private double calculateTurretPower(double targetDeg, double currentDeg) {
        long nowNs = System.nanoTime();
        double error = normalizeDegrees(targetDeg - currentDeg);

        if (Math.abs(error) <= angleToleranceDeg) {
            resetController();
            return 0.0;
        }

        if (lastTimeNs < 0) {
            lastTimeNs = nowNs;
            lastError = error;
            return clamp(kP * error, -maxPower, maxPower);
        }

        double dt = (nowNs - lastTimeNs) * 1e-9;
        lastTimeNs = nowNs;
        if (dt <= 0.0) {
            return turretPow;
        }

        integral = clamp(integral + error * dt, -maxIntegral, maxIntegral);
        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = kP * error + kI * integral + kD * derivative;
        return clamp(output, -maxPower, maxPower);
    }

    private void resetController() {
        integral = 0.0;
        lastError = 0.0;
        lastTimeNs = -1L;
    }

    private static double encoderTicksToDegrees(int ticks) {
        return clampToTurretRange(normalizeDegrees((ticks / ticksPerRev) * 360.0 / gearRatio + encoderOffsetDeg));
    }

    private static double normalizeDegrees(double angleDeg) {
        while (angleDeg > 180.0) angleDeg -= 360.0;
        while (angleDeg <= -180.0) angleDeg += 360.0;
        return angleDeg;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double clampToTurretRange(double angleDeg) {
        return clamp(angleDeg, minAngleDeg, maxAngleDeg);
    }
}
