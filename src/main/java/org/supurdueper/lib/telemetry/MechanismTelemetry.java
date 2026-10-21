package org.supurdueper.lib.telemetry;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Tuning telemetry of one mechanism: its motors' signals, a {@code Config} JSON entry (gains, ratio, current limits,
 * output settings, tolerance) and the team's {@code MechanismOK} flag (dashboard "Tuning/Mechanism OK/<name>", set
 * false while the mechanism is known to be faulty so its data is excluded from tuning).
 */
public final class MechanismTelemetry {

    private final String name;
    private final String loop;
    private final DataLog log;
    private final String prefix;
    private final List<MotorTelemetry> motors = new ArrayList<>();
    private final List<Integer> leaders = new ArrayList<>();
    private final Map<Integer, String> followers = new LinkedHashMap<>();
    private final Map<Integer, String> modules = new LinkedHashMap<>();
    private final Map<String, DoubleSupplier> extras = new LinkedHashMap<>();
    private BaseStatusSignal[] refreshSignals = new BaseStatusSignal[0];
    private Supplier<TalonFXConfiguration> config = TalonFXConfiguration::new;
    private DoubleSupplier tolerance = () -> 0;
    private final StringLogEntry configEntry;
    private final BooleanLogEntry mechanismOkEntry;
    private final String mechanismOkKey;
    private boolean configChanged = false;

    MechanismTelemetry(DataLog log, String name, String loop) {
        this.name = name;
        this.loop = loop;
        this.log = log;
        prefix = "/Tuning/" + name + "/";
        configEntry = new StringLogEntry(log, prefix + "Config", "", "json");
        mechanismOkEntry = new BooleanLogEntry(log, prefix + "MechanismOK");
        mechanismOkKey = "Tuning/Mechanism OK/" + name;
        SmartDashboard.setDefaultBoolean(mechanismOkKey, true);
    }

    /** Leader of a velocity, position or open-loop mechanism. */
    public MechanismTelemetry addLeader(TalonFX motor) {
        leaders.add(motor.getDeviceID());
        return add(motor, "position".equals(loop) ? MotorTelemetry.Role.POSITION_LEADER : MotorTelemetry.Role.LEADER);
    }

    public MechanismTelemetry addFollower(TalonFX motor, boolean opposed) {
        followers.put(motor.getDeviceID(), opposed ? "Opposed" : "Aligned");
        return add(motor, MotorTelemetry.Role.FOLLOWER);
    }

    /** Swerve module motor; speeds and angles come from the swerve state arrays. */
    public MechanismTelemetry addSwerveMotor(TalonFX motor, String module) {
        leaders.add(motor.getDeviceID());
        modules.put(motor.getDeviceID(), module);
        return add(motor, MotorTelemetry.Role.SWERVE);
    }

    private MechanismTelemetry add(TalonFX motor, MotorTelemetry.Role role) {
        MotorTelemetry motorTelemetry = new MotorTelemetry(log, prefix, motor, role);
        motors.add(motorTelemetry);
        List<BaseStatusSignal> signals = new ArrayList<>(List.of(refreshSignals));
        signals.addAll(motorTelemetry.refreshSignals());
        refreshSignals = signals.toArray(BaseStatusSignal[]::new);
        return this;
    }

    /**
     * @param config the configuration applied to the motors (read when the Config entry is written)
     * @param tolerance at-setpoint tolerance in mechanism rotations (position) or rotations per second (velocity)
     */
    public MechanismTelemetry withConfig(Supplier<TalonFXConfiguration> config, DoubleSupplier tolerance) {
        this.config = config;
        this.tolerance = tolerance;
        return this;
    }

    /** Additional numeric Config field, e.g. a threshold the analysis needs. */
    public MechanismTelemetry withExtra(String key, DoubleSupplier value) {
        extras.put(key, value);
        return this;
    }

    /** Write the Config entry on the next loop (gains or limits were applied). */
    public void markConfigChanged() {
        configChanged = true;
    }

    /**
     * @param medium 10 Hz items (control mode, MechanismOK)
     * @param slow 1 Hz items (temperature, Config)
     */
    void update(boolean medium, boolean slow) {
        BaseStatusSignal.refreshAll(refreshSignals);
        for (MotorTelemetry motor : motors) {
            motor.update(medium, slow);
        }
        if (medium) {
            mechanismOkEntry.update(SmartDashboard.getBoolean(mechanismOkKey, true));
        }
        if (slow || configChanged) {
            configEntry.update(configJson());
            configChanged = false;
        }
    }

    private String configJson() {
        TalonFXConfiguration c = config.get();
        Slot0Configs gains = c.Slot0;
        CurrentLimitsConfigs limits = c.CurrentLimits;
        StringBuilder json = new StringBuilder(640).append('{');
        string(json, "mechanism", name);
        string(json, "loop", loop);
        json.append("\"leaders\":").append(leaders).append(',');
        idMap(json, "followers", followers);
        idMap(json, "modules", modules);
        json.append("\"gains\":{");
        number(json, "kP", gains.kP);
        number(json, "kI", gains.kI);
        number(json, "kD", gains.kD);
        number(json, "kS", gains.kS);
        number(json, "kV", gains.kV);
        number(json, "kA", gains.kA);
        number(json, "kG", gains.kG);
        json.setLength(json.length() - 1);
        json.append("},");
        string(json, "gravityType", gains.GravityType.toString());
        number(json, "sensorToMechanismRatio", c.Feedback.SensorToMechanismRatio);
        number(json, "statorLimit", limits.StatorCurrentLimit);
        bool(json, "statorLimitEnable", limits.StatorCurrentLimitEnable);
        number(json, "supplyLimit", limits.SupplyCurrentLimit);
        bool(json, "supplyLimitEnable", limits.SupplyCurrentLimitEnable);
        number(json, "supplyLowerLimit", limits.SupplyCurrentLowerLimit);
        number(json, "supplyLowerTime", limits.SupplyCurrentLowerTime);
        string(json, "inverted", c.MotorOutput.Inverted.toString());
        string(json, "neutralMode", c.MotorOutput.NeutralMode.toString());
        number(json, "tolerance", tolerance.getAsDouble());
        for (Map.Entry<String, DoubleSupplier> extra : extras.entrySet()) {
            number(json, extra.getKey(), extra.getValue().getAsDouble());
        }
        json.setLength(json.length() - 1);
        return json.append('}').toString();
    }

    private static void string(StringBuilder json, String key, String value) {
        json.append('"').append(key).append("\":\"").append(value).append("\",");
    }

    private static void number(StringBuilder json, String key, double value) {
        json.append('"').append(key).append("\":").append(Double.isFinite(value) ? Double.toString(value) : "null");
        json.append(',');
    }

    private static void bool(StringBuilder json, String key, boolean value) {
        json.append('"').append(key).append("\":").append(value).append(',');
    }

    private static void idMap(StringBuilder json, String key, Map<Integer, String> values) {
        json.append('"').append(key).append("\":{");
        for (Map.Entry<Integer, String> value : values.entrySet()) {
            json.append('"')
                    .append(value.getKey())
                    .append("\":\"")
                    .append(value.getValue())
                    .append("\",");
        }
        if (!values.isEmpty()) {
            json.setLength(json.length() - 1);
        }
        json.append("},");
    }
}
