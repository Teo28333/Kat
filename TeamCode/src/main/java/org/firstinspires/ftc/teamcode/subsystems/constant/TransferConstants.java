package org.firstinspires.ftc.teamcode.subsystems.constant;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TransferConstants {
    public static String transferServoName = "transfer";

    public static double restPosition = 0.0;
    public static double activePosition = 1.0;

    public static double intakeBeforeKickSec = 0.5;
    public static double kickerActiveSec = 0.1;
    public static double waitAfterKickSec = 0.125;
}
