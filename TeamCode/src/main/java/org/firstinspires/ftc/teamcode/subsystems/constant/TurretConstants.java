package org.firstinspires.ftc.teamcode.subsystems.constant;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TurretConstants {
    public static String turretMotorName = "turret";

    public static double
            kP = 0.025,
            kI = 0.00001,
            kD = 0.00001;

    public static double maxPower = 1.0;
    public static double angleToleranceDeg = 2.0;
    public static double maxIntegral = 250.0;

    public static double minAngleDeg = -100.0;
    public static double maxAngleDeg = 100.0;

    public static double ticksPerRev = 383.60;
    public static double gearRatio = 120.0 / 42.0;
    public static double encoderOffsetDeg = 0.0;
}
