package org.firstinspires.ftc.teamcode.subsystems.constant;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class IntakeConstants {
    public static String frontRollerMotorName = "frontRoller";
    public static String conveyorMotorName = "conveyor";

    public static double frontRollerIntakePower = 1.0;
    public static double conveyorIntakePower = 1.0;

    public static double frontRollerOuttakePower = -1.0;
    public static double conveyorOuttakePower = -1.0;

    public static double stopPower = 0.0;
}
