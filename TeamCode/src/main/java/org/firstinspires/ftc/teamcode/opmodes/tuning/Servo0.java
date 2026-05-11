package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Servo0 extends OpMode {
    private Servo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(0.5);
    }

    @Override
    public void loop() {
        servo.setPosition(0.5);
    }
}
