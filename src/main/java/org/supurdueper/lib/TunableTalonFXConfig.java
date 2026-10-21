package org.supurdueper.lib;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.List;
import java.util.function.Supplier;
import org.supurdueper.robot2026.Constants;

/**
 * Tuning mode: dashboard current limits, and optionally Slot0 gains, for TalonFX that share one configuration; an edit
 * is applied to every motor of the group and recorded in the configuration object. Keys under /Tuning/<name>/: "Stator
 * Limit", "Supply Limit" (0 disables the limit), "Supply Lower Limit", "Supply Lower Time" and, with gains, "Kp", "Ki",
 * "Kd", "Ks", "Kv", "Ka".
 */
public final class TunableTalonFXConfig {

    private final TalonFXConfiguration config;
    private final Supplier<List<TalonFX>> motors;
    private final Runnable onApplied;
    private final LoggedTunableNumber[] limits;
    private final LoggedTunableNumber[] gains;
    private final int id = System.identityHashCode(this);

    /**
     * @param config configuration the motors were configured with; edits are written into it
     * @param motors motors to apply edits to
     * @param tuneGains also offer the Slot0 gains
     * @param onApplied called after an edit was applied (e.g. to log the new configuration)
     */
    public TunableTalonFXConfig(
            String name,
            TalonFXConfiguration config,
            Supplier<List<TalonFX>> motors,
            boolean tuneGains,
            Runnable onApplied) {
        this.config = config;
        this.motors = motors;
        this.onApplied = onApplied;
        CurrentLimitsConfigs currentLimits = config.CurrentLimits;
        limits = new LoggedTunableNumber[] {
            new LoggedTunableNumber(
                    name + "/Stator Limit",
                    currentLimits.StatorCurrentLimitEnable ? currentLimits.StatorCurrentLimit : 0),
            new LoggedTunableNumber(
                    name + "/Supply Limit",
                    currentLimits.SupplyCurrentLimitEnable ? currentLimits.SupplyCurrentLimit : 0),
            new LoggedTunableNumber(name + "/Supply Lower Limit", currentLimits.SupplyCurrentLowerLimit),
            new LoggedTunableNumber(name + "/Supply Lower Time", currentLimits.SupplyCurrentLowerTime)
        };
        Slot0Configs slot0 = config.Slot0;
        gains = tuneGains
                ? new LoggedTunableNumber[] {
                    new LoggedTunableNumber(name + "/Kp", slot0.kP),
                    new LoggedTunableNumber(name + "/Ki", slot0.kI),
                    new LoggedTunableNumber(name + "/Kd", slot0.kD),
                    new LoggedTunableNumber(name + "/Ks", slot0.kS),
                    new LoggedTunableNumber(name + "/Kv", slot0.kV),
                    new LoggedTunableNumber(name + "/Ka", slot0.kA)
                }
                : new LoggedTunableNumber[0];
        // Record the defaults so only dashboard edits are applied
        for (LoggedTunableNumber limit : limits) {
            limit.hasChanged(id);
        }
        for (LoggedTunableNumber gain : gains) {
            gain.hasChanged(id + 1);
        }
    }

    /** Call periodically; applies changed dashboard values in tuning mode. */
    public void update() {
        if (!Constants.tuningMode) {
            return;
        }
        LoggedTunableNumber.ifChanged(
                id,
                values -> {
                    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                            .withStatorCurrentLimit(values[0])
                            .withStatorCurrentLimitEnable(values[0] > 0)
                            .withSupplyCurrentLimit(values[1])
                            .withSupplyCurrentLimitEnable(values[1] > 0)
                            .withSupplyCurrentLowerLimit(values[2])
                            .withSupplyCurrentLowerTime(values[3]);
                    config.CurrentLimits = currentLimits;
                    motors.get().forEach(motor -> motor.getConfigurator().apply(currentLimits));
                    onApplied.run();
                },
                limits);
        if (gains.length > 0) {
            LoggedTunableNumber.ifChanged(
                    id + 1,
                    values -> {
                        Slot0Configs slot0 = config.Slot0.withKP(values[0])
                                .withKI(values[1])
                                .withKD(values[2])
                                .withKS(values[3])
                                .withKV(values[4])
                                .withKA(values[5]);
                        motors.get().forEach(motor -> motor.getConfigurator().apply(slot0));
                        onApplied.run();
                    },
                    gains);
        }
    }
}
