package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TurretSS;
import org.firstinspires.ftc.teamcode.subsystems.constant.TurretConstants;

import java.util.Locale;

@Configurable
@TeleOp(name = "Turret Tuning", group = "Tuning")
public class TurretTuning extends OpMode {
    public static double targetAngleDeg = 0.0;
    public static boolean turretEnabled = true;

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private TurretSS turret;

    @Override
    public void init() {
        turret = new TurretSS(hardwareMap, telemetry);
        PanelsConfigurables.INSTANCE.refreshClass(this);
        PanelsConfigurables.INSTANCE.refreshClass(new TurretConstants());
    }

    @Override
    public void loop() {
        turret.read();

        if (turretEnabled) {
            turret.aimToAngleDeg(targetAngleDeg);
        } else {
            turret.stopTurret();
        }

        turret.write();
        turret.telemetry();

        double errorDeg = targetAngleDeg - turret.getCurrentAngleDeg();

        telemetry.addData("Target angle deg", "%.1f", targetAngleDeg);
        telemetry.addData("Current angle deg", "%.1f", turret.getCurrentAngleDeg());
        telemetry.addData("Error deg", "%.1f", errorDeg);
        telemetry.addData("Power", "%.3f", turret.getTurretPow());
        telemetry.addData("At target", turret.isAtTarget());
        telemetry.update();

        panelsTelemetry.addData("Turret Target Deg", round(targetAngleDeg));
        panelsTelemetry.addData("Turret Current Deg", round(turret.getCurrentAngleDeg()));
        panelsTelemetry.addData("Turret Error Deg", round(errorDeg));
        panelsTelemetry.addData("Turret Power", round(turret.getTurretPow()));
        panelsTelemetry.addData("Turret At Target", turret.isAtTarget());
        panelsTelemetry.addData("Turret Enabled", turretEnabled);
        panelsTelemetry.update();
    }

    @Override
    public void stop() {
        turret.stopTurret();
        turret.write();
    }

    private static String round(double value) {
        return String.format(Locale.US, "%.2f", value);
    }
}
