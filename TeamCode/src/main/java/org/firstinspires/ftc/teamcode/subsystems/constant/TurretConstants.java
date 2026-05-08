package org.firstinspires.ftc.teamcode.subsystems.constant;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TurretConstants {
    public static String turretMotorName = "turret";

    public static double
            kP = 0.0,
            kI = 0.0,
            kD = 0.0;

    public static double maxPower = 0.5;
    public static double angleToleranceDeg = 2.0;
    public static double maxIntegral = 250.0;

    public static double minAngleDeg = -135.0;
    public static double maxAngleDeg = 135.0;

    public static double ticksPerRev = 537.7;
    public static double gearRatio = 120.0 / 42.0;
    public static double encoderOffsetDeg = 0.0;
}
