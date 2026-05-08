package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Alliance;

@Autonomous(name = "Red Preload Shoot", group = "Auto")
public class RedPreloadShoot extends PreloadShootAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
