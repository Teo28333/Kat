package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Alliance;

@TeleOp(name = "Blue TeleOp", group = "TeleOp")
public class Blue extends AllianceTeleOp {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
