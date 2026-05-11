package org.firstinspires.ftc.teamcode.subsystems.constant;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterConstants {
    public static String shooterMotorName = "shooter";

    public static double
            kP = 0.00001,
            kI = 0.0,
            kD = 0.0,
            kF = 0.000189,
            kS = 0.0;

    public static double MAX_INTEGRAL = 5000;

    public static double nominalVoltage = 12.5;

    public static double velocityToleranceRPM = 75.0;
}
