package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
@TeleOp(name = "Wheel Debug", group = "Tuning")
public class WheelDebug extends OpMode {
    public static double testPower = 0.25;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PanelsConfigurables.INSTANCE.refreshClass(this);
    }

    @Override
    public void loop() {
        double frontLeftPower = gamepad1.a ? testPower : 0.0;
        double frontRightPower = gamepad1.b ? testPower : 0.0;
        double backLeftPower = gamepad1.x ? testPower : 0.0;
        double backRightPower = gamepad1.y ? testPower : 0.0;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        telemetry.addData("Test power", "%.2f", testPower);
        telemetry.addData("A", "frontLeft");
        telemetry.addData("B", "frontRight");
        telemetry.addData("X", "backLeft");
        telemetry.addData("Y", "backRight");
        telemetry.addData("frontLeft power", "%.2f", frontLeftPower);
        telemetry.addData("frontRight power", "%.2f", frontRightPower);
        telemetry.addData("backLeft power", "%.2f", backLeftPower);
        telemetry.addData("backRight power", "%.2f", backRightPower);
        telemetry.addData("frontLeft ticks", frontLeft.getCurrentPosition());
        telemetry.addData("frontRight ticks", frontRight.getCurrentPosition());
        telemetry.addData("backLeft ticks", backLeft.getCurrentPosition());
        telemetry.addData("backRight ticks", backRight.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        stopAllMotors();
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        frontLeft.setZeroPowerBehavior(zeroPowerBehavior);
        frontRight.setZeroPowerBehavior(zeroPowerBehavior);
        backLeft.setZeroPowerBehavior(zeroPowerBehavior);
        backRight.setZeroPowerBehavior(zeroPowerBehavior);
    }

    private void stopAllMotors() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }
}
