package org.supurdueper.lib.telemetry;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import java.util.ArrayList;
import java.util.List;

/**
 * Tuning signals of one TalonFX under {@code /Tuning/<mechanism>/<CAN id>/}. Values are rounded to a fixed resolution
 * and written to the DataLog only when they change, so readers hold the last value until the next record.
 */
final class MotorTelemetry {

    /** Which signals a motor logs. */
    enum Role {
        /** Output, speed, closed-loop reference, currents, control mode. */
        LEADER,
        /** LEADER plus position. */
        POSITION_LEADER,
        /** Speed and currents; a follower's output mirrors its leader. */
        FOLLOWER,
        /** Output, currents, control mode; speed and angle come from the swerve state arrays (odometry thread). */
        SWERVE
    }

    // Mechanism signals are requested at this rate; the swerve keeps the rates it sets for odometry
    private static final double kSignalFrequencyHz = 100.0;

    private record Channel(StatusSignal<?> signal, DoubleLogEntry entry, double resolution) {
        void update() {
            entry.update(Math.round(signal.getValueAsDouble() / resolution) * resolution);
        }
    }

    private final List<Channel> channels = new ArrayList<>();
    private final List<BaseStatusSignal> refreshSignals = new ArrayList<>();
    private final Channel temp;
    private final StatusSignal<ControlModeValue> controlMode;
    private final StringLogEntry controlModeEntry;

    MotorTelemetry(DataLog log, String mechanismPrefix, TalonFX motor, Role role) {
        String prefix = mechanismPrefix + motor.getDeviceID() + "/";
        if (role != Role.FOLLOWER) {
            add(log, prefix + "Voltage", motor.getMotorVoltage(false), 0.01);
        }
        if (role != Role.SWERVE) {
            add(log, prefix + "Velocity", motor.getVelocity(false), 0.001);
        }
        if (role == Role.POSITION_LEADER) {
            add(log, prefix + "Position", motor.getPosition(false), 0.0001);
        }
        if (role == Role.LEADER || role == Role.POSITION_LEADER) {
            add(log, prefix + "Reference", motor.getClosedLoopReference(false), 0.0001);
        }
        add(log, prefix + "StatorCurrent", motor.getStatorCurrent(false), 0.1);
        add(log, prefix + "SupplyCurrent", motor.getSupplyCurrent(false), 0.1);
        if (role != Role.SWERVE) {
            BaseStatusSignal.setUpdateFrequencyForAll(
                    kSignalFrequencyHz, channels.stream().map(Channel::signal).toArray(BaseStatusSignal[]::new));
        }
        temp = new Channel(motor.getDeviceTemp(false), new DoubleLogEntry(log, prefix + "Temp"), 0.1);
        if (role == Role.FOLLOWER) {
            controlMode = null;
            controlModeEntry = null;
        } else {
            controlMode = motor.getControlMode(false);
            controlModeEntry = new StringLogEntry(log, prefix + "ControlMode");
            refreshSignals.add(controlMode);
        }
    }

    private void add(DataLog log, String name, StatusSignal<?> signal, double resolution) {
        channels.add(new Channel(signal, new DoubleLogEntry(log, name), resolution));
        refreshSignals.add(signal);
    }

    /** Signals the mechanism refreshes every loop before {@link #update}. */
    List<BaseStatusSignal> refreshSignals() {
        return refreshSignals;
    }

    /** Writes changed values; temperature only on slow loops (1 Hz). */
    void update(boolean slow) {
        for (Channel channel : channels) {
            channel.update();
        }
        if (controlMode != null) {
            controlModeEntry.update(controlMode.getValue().toString());
        }
        if (slow) {
            temp.signal().refresh();
            temp.update();
        }
    }
}
