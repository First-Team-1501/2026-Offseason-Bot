package org.supurdueper.lib.telemetry;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Log-based tuning telemetry for every motor and the power system, written directly to the WPILib DataLog under
 * {@code /Tuning} (not through the DogLog queue, not published to NT). Consumed by {@code tools/tuning}; the key
 * contract is in the vault spec "Offseason Bot/Continuous Tuning System".
 */
public final class TuningTelemetry {

    private static final int kSlowLoops = 50; // 1 Hz at the 20 ms loop
    private static final int kMediumLoops = 5; // 10 Hz: control mode, MechanismOK
    private static final int kDisabledLoops = 25; // 2 Hz motor signals while disabled (the rio CPU is saturated)
    private static final int kPowerCheckLoop = 50;
    private static final double kMinPdhVoltage = 1.0;

    private static final Map<String, MechanismTelemetry> mechanisms = new LinkedHashMap<>();
    private static int loopCount = 0;
    private static boolean powerInitialized = false;
    private static boolean pdhChecked = false;
    private static PowerDistribution pdh;
    private static DoubleLogEntry batteryVoltage;
    private static BooleanLogEntry brownedOut;
    private static BooleanLogEntry enabled;
    private static StringLogEntry mode;
    private static DoubleLogEntry pdhTotalCurrent;
    private static DoubleLogEntry pdhVoltage;
    private static DoubleArrayLogEntry pdhCurrents;
    private static DoubleLogEntry telemetryTime;

    private TuningTelemetry() {}

    static DataLog log() {
        return DataLogManager.getLog();
    }

    /** The mechanism's telemetry, created on first use. */
    public static synchronized MechanismTelemetry mechanism(String name, String loop) {
        return mechanisms.computeIfAbsent(name, key -> new MechanismTelemetry(log(), key, loop));
    }

    /** Call once per main loop. */
    public static void periodic() {
        double start = Timer.getFPGATimestamp();
        loopCount++;
        boolean slow = loopCount % kSlowLoops == 0;
        boolean medium = loopCount % kMediumLoops == 0;
        if (DriverStation.isEnabled() || loopCount % kDisabledLoops == 0) {
            for (MechanismTelemetry mechanism : mechanisms.values()) {
                mechanism.update(medium, slow);
            }
        }
        logPower();
        telemetryTime.update(Math.round((Timer.getFPGATimestamp() - start) * 1e5) / 100.0);
    }

    private static void logPower() {
        if (!powerInitialized) {
            initPower();
        }
        batteryVoltage.update(Math.round(RobotController.getBatteryVoltage() * 100) / 100.0);
        brownedOut.update(RobotController.isBrownedOut());
        enabled.update(DriverStation.isEnabled());
        mode.update(
                DriverStation.isDisabled()
                        ? "disabled"
                        : DriverStation.isAutonomous() ? "auto" : DriverStation.isTest() ? "test" : "teleop");
        if (pdh == null) {
            return;
        }
        if (!pdhChecked) {
            // Probe once, after its status frames had time to arrive: every read of an absent module reports a CAN
            // error with a stack trace, which overran the loop
            if (loopCount < kPowerCheckLoop) {
                return;
            }
            pdhChecked = true;
            if (pdh.getVoltage() < kMinPdhVoltage) {
                DataLogManager.log("TuningTelemetry: no power distribution module data, PDH logging off");
                pdh.close();
                pdh = null;
                return;
            }
        }
        double voltage = pdh.getVoltage();
        pdhVoltage.update(Math.round(voltage * 100) / 100.0);
        pdhTotalCurrent.update(Math.round(pdh.getTotalCurrent() * 10) / 10.0);
        double[] currents = pdh.getAllCurrents();
        for (int i = 0; i < currents.length; i++) {
            currents[i] = Math.round(currents[i] * 10) / 10.0;
        }
        pdhCurrents.update(currents);
    }

    private static void initPower() {
        powerInitialized = true;
        DataLog log = log();
        batteryVoltage = new DoubleLogEntry(log, "/Tuning/Power/BatteryVoltage");
        brownedOut = new BooleanLogEntry(log, "/Tuning/Power/BrownedOut");
        enabled = new BooleanLogEntry(log, "/Tuning/Power/Enabled");
        mode = new StringLogEntry(log, "/Tuning/Power/Mode");
        pdhTotalCurrent = new DoubleLogEntry(log, "/Tuning/Power/PDH/TotalCurrent");
        pdhVoltage = new DoubleLogEntry(log, "/Tuning/Power/PDH/Voltage");
        pdhCurrents = new DoubleArrayLogEntry(log, "/Tuning/Power/PDH/Currents");
        telemetryTime = new DoubleLogEntry(log, "/Tuning/Telemetry Time (ms)");
        try {
            pdh = new PowerDistribution();
        } catch (RuntimeException e) {
            DataLogManager.log("TuningTelemetry: power distribution module not found: " + e.getMessage());
            pdh = null;
        }
    }
}
