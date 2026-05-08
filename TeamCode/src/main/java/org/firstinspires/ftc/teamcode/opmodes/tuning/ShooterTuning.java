package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.constant.ShooterConstants;

import java.util.Locale;

@Configurable
@TeleOp(name = "Shooter Tuning", group = "Tuning")
public class ShooterTuning extends OpMode {
    public static double targetRPM = 3000.0;
    public static boolean shooterEnabled = true;

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private ShooterSS shooter;

    @Override
    public void init() {
        shooter = new ShooterSS(hardwareMap);
        PanelsConfigurables.INSTANCE.refreshClass(this);
        PanelsConfigurables.INSTANCE.refreshClass(new ShooterConstants());
    }

    @Override
    public void loop() {
        shooter.read();

        if (shooterEnabled) {
            shooter.spinShooterRPM(targetRPM);
        } else {
            shooter.stopShooter();
        }

        shooter.write();

        double errorRPM = shooter.getTargetRPM() - shooter.getCurrentVelRPM();

        telemetry.addData("Target RPM", "%.1f", shooter.getTargetRPM());
        telemetry.addData("Current RPM", "%.1f", shooter.getCurrentVelRPM());
        telemetry.addData("Error RPM", "%.1f", errorRPM);
        telemetry.addData("Power", "%.3f", shooter.getShooterPow());
        telemetry.addData("Ready", shooter.isReady());
        telemetry.update();

        panelsTelemetry.addData("Shooter Target RPM", round(shooter.getTargetRPM()));
        panelsTelemetry.addData("Shooter Current RPM", round(shooter.getCurrentVelRPM()));
        panelsTelemetry.addData("Shooter Error RPM", round(errorRPM));
        panelsTelemetry.addData("Shooter Power", round(shooter.getShooterPow()));
        panelsTelemetry.addData("Shooter Ready", shooter.isReady());
        panelsTelemetry.addData("Shooter Enabled", shooterEnabled);
        panelsTelemetry.update();
    }

    @Override
    public void stop() {
        shooter.stopShooter();
        shooter.update();
    }

    private static String round(double value) {
        return String.format(Locale.US, "%.2f", value);
    }
}
