package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;

public abstract class AllianceTeleOp extends OpMode {

    private Robot robot;

    protected abstract Alliance alliance();

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, alliance());
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.update(gamepad1, gamepad2);
    }
}
