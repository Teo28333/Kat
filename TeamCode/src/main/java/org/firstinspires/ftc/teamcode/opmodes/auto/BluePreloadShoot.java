package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Alliance;

@Autonomous(name = "Blue Preload Shoot", group = "Auto")
public class BluePreloadShoot extends PreloadShootAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
