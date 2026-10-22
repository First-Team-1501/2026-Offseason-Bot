package org.supurdueper.lib.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.supurdueper.lib.TalonFXFactory;
import org.supurdueper.lib.TunableTalonFXConfig;
import org.supurdueper.lib.telemetry.MechanismTelemetry;
import org.supurdueper.lib.telemetry.TuningTelemetry;
import org.supurdueper.robot2026.CanId;

public abstract class TalonFXSubsystem extends SubsystemBase {

    public VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false);
    public NeutralOut stopRequest = new NeutralOut();
    public TalonFX motor;
    public TalonFX followerMotor;
    public TalonFXConfiguration config;
    protected final List<TalonFX> followers = new ArrayList<>();
    protected MechanismTelemetry telemetry;
    // Current limits, adjustable live in tuning mode
    private final TunableTalonFXConfig tunableCurrentLimits;

    public TalonFXSubsystem() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(inverted() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive)
                .withNeutralMode(brakeMode() ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        config = TalonFXFactory.getDefaultConfig()
                .withCurrentLimits(currentLimits())
                .withMotorOutput(motorOutputConfigs);
        tunableCurrentLimits = new TunableTalonFXConfig(
                getName(), config, this::allMotors, false, () -> telemetry.markConfigChanged());
    }

    public Command setVoltage(DoubleSupplier voltage) {
        return run(() -> runVoltage(Volts.of(voltage.getAsDouble())));
    }

    protected void runVoltage(Voltage volts) {
        if (outputInhibited()) {
            stop();
            return;
        }
        motor.setControl(voltageRequest.withOutput(volts));
    }

    protected void stop() {
        motor.setControl(stopRequest);
    }

    /**
     * True while the motors must stay neutral (interlocks, defense mode). Output requests are replaced by NeutralOut;
     * commands keep running and take effect again when this returns false.
     */
    protected boolean outputInhibited() {
        return false;
    }

    @Override
    public void periodic() {
        // A command that ended leaves its last request active in the Talon
        if (outputInhibited()) {
            stop();
        }
    }

    protected void configureMotors() {
        motor = TalonFXFactory.createConfigTalon(canIdLeader(), config);
        telemetry = TuningTelemetry.mechanism(getName(), telemetryLoop())
                .withConfig(() -> config, this::telemetryTolerance)
                .addLeader(motor);
        if (canIdFollower() != null) {
            followerMotor = addFollower(canIdFollower(), followerInverted());
        }
    }

    /** Creates a permanent follower with the leader's configuration and registers its tuning telemetry. */
    protected TalonFX addFollower(CanId followerId, boolean opposeLeader) {
        TalonFX follower = TalonFXFactory.createPermanentFollowerTalon(followerId, motor, opposeLeader);
        followers.add(follower);
        telemetry.addFollower(follower, opposeLeader);
        return follower;
    }

    /** Tuning telemetry loop type: velocity, position or openloop. */
    protected String telemetryLoop() {
        return "openloop";
    }

    /** At-setpoint tolerance in mechanism units, for the tuning analysis. */
    protected double telemetryTolerance() {
        return 0;
    }

    /** Tuning mode: apply dashboard current limits to the leader and followers. */
    protected void updateTunableCurrentLimits() {
        tunableCurrentLimits.update();
    }

    /** Leader and followers. */
    protected List<TalonFX> allMotors() {
        List<TalonFX> motors = new ArrayList<>(followers);
        motors.add(0, motor);
        return motors;
    }

    public abstract CanId canIdLeader();

    public abstract CanId canIdFollower();

    public abstract boolean followerInverted();

    public abstract CurrentLimitsConfigs currentLimits();

    public abstract boolean inverted();

    public abstract boolean brakeMode();
}
