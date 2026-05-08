package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.subsystems.constant.IntakeConstants.*;

public class IntakeSS implements SubsystemInterface {

    private DcMotorEx frontRollerMotor;
    private DcMotorEx conveyorMotor;
    private Telemetry telemetry;

    private double frontRollerPow = 0.0;
    private double conveyorPow = 0.0;
    private double frontRollerCurrentVel = 0.0;
    private double conveyorCurrentVel = 0.0;

    public IntakeSS(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public IntakeSS(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        frontRollerMotor = hardwareMap.get(DcMotorEx.class, frontRollerMotorName);
        conveyorMotor = hardwareMap.get(DcMotorEx.class, conveyorMotorName);
    }

    public void read() {
        frontRollerCurrentVel = frontRollerMotor.getVelocity();
        conveyorCurrentVel = conveyorMotor.getVelocity();
    }

    public void write() {
        frontRollerMotor.setPower(frontRollerPow);
        conveyorMotor.setPower(conveyorPow);
    }

    public void telemetry() {
        if (telemetry == null) {
            return;
        }

        telemetry.addData("Front roller power", "%.2f", frontRollerPow);
        telemetry.addData("Conveyor power", "%.2f", conveyorPow);
        telemetry.addData("Front roller velocity", "%.1f", frontRollerCurrentVel);
        telemetry.addData("Conveyor velocity", "%.1f", conveyorCurrentVel);
    }

    public void update() {
        read();
        write();
        telemetry();
    }

    public void intake() {
        setIntakePower(frontRollerIntakePower, conveyorIntakePower);
    }

    public void outtake() {
        setIntakePower(frontRollerOuttakePower, conveyorOuttakePower);
    }

    public void stopIntake() {
        setIntakePower(stopPower, stopPower);
    }

    private void setIntakePower(double frontRollerPower, double conveyorPower) {
        frontRollerPow = frontRollerPower;
        conveyorPow = conveyorPower;
    }

    public double getFrontRollerPow() {
        return frontRollerPow;
    }

    public double getConveyorPow() {
        return conveyorPow;
    }

    public double getFrontRollerCurrentVel() {
        return frontRollerCurrentVel;
    }

    public double getConveyorCurrentVel() {
        return conveyorCurrentVel;
    }
}
