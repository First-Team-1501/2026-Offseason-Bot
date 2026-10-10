package org.supurdueper.lib.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.supurdueper.lib.LoggedTunableNumber;
import org.supurdueper.robot2026.Constants;

public abstract class VelocitySubsystem extends TalonFXSubsystem {

    // Tunable numbers
    private final LoggedTunableNumber kp;
    private final LoggedTunableNumber ks;
    private final LoggedTunableNumber kv;
    private final List<LoggedTunableNumber> pidGains;
    // Voltage closed loop with FOC disabled: no Phoenix Pro license required
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(false);
    protected final AngularVelocity velocityTolerance;
    private final SysIdRoutine sysIdRoutine;
    private final Trigger atVelocity = new Trigger(this::isAtVelocity);

    protected StatusSignal<AngularVelocity> motorVelocitySignal;
    protected StatusSignal<Double> motorSetpointSignal;

    public Command sysIdQuasistaticFoward() {
        return sysIdRoutine.quasistatic(Direction.kForward);
    }

    public Command sysIdQuasistaticReverse() {
        return sysIdRoutine.quasistatic(Direction.kReverse);
    }

    public Command sysIdDynamicFoward() {
        return sysIdRoutine.dynamic(Direction.kForward);
    }

    public Command sysIdDynamicReverse() {
        return sysIdRoutine.dynamic(Direction.kReverse);
    }

    public Command goToVelocity(Supplier<AngularVelocity> angularvelocity) {
        return run(() -> setVelocity(angularvelocity.get()));
    }

    public Command goToVelocityBlocking(Supplier<AngularVelocity> angularvelocity) {
        return goToVelocity(angularvelocity).andThen(Commands.waitUntil(this::isAtVelocity));
    }

    public Trigger isAtVelocityTrigger() {
        return atVelocity;
    }

    protected void setVelocity(AngularVelocity angularvelocity) {
        motor.setControl(velocityRequest.withVelocity(angularvelocity));
    }

    protected void setVelocity(double angularvelocity) {
        motor.setControl(velocityRequest.withVelocity(angularvelocity));
    }

    protected AngularVelocity getPosition() {
        return motorVelocitySignal.getValue();
    }

    protected AngularVelocity getVelocity() {
        return Units.RotationsPerSecond.of(motorVelocitySignal.getValueAsDouble());
    }

    public AngularVelocity getSetpoint() {
        return Units.RotationsPerSecond.of(motorSetpointSignal.getValueAsDouble());
    }

    protected boolean isAtVelocity() {
        return (getSetpoint().minus(getVelocity())).abs(RotationsPerSecond)
                < (velocityTolerance).abs(RotationsPerSecond);
    }

    public VelocitySubsystem() {
        super();

        // Setup tunable pid gains
        String name = this.getName();
        kp = new LoggedTunableNumber(name + "/Kp");
        ks = new LoggedTunableNumber(name + "/Ks");
        kv = new LoggedTunableNumber(name + "/Kv");
        Slot0Configs gains = pidGains();
        kp.initDefault(gains.kP);
        ks.initDefault(gains.kS);
        kv.initDefault(gains.kV);
        pidGains = new ArrayList<>();
        pidGains.addAll(List.of(kp, ks, kv));
        velocityTolerance = velocityTolerance();
        sysIdRoutine = sysIdConfig();
        // Add motion magic items to config
        config = config.withSlot0(gains);
    }

    @Override
    public void periodic() {
        if (Constants.tuningMode) {
            for (LoggedTunableNumber gain : pidGains) {
                if (gain.hasChanged(hashCode())) {
                    // Send new PID gains to talon
                    Slot0Configs slot0config =
                            new Slot0Configs().withKP(kp.get()).withKS(ks.get()).withKV(kv.get());
                    motor.getConfigurator().apply(config.withSlot0(slot0config));
                    break;
                }
            }
        }
        StatusSignal.refreshAll(motorVelocitySignal, motorSetpointSignal);
        super.periodic();
    }

    @Override
    protected void configureMotors() {
        super.configureMotors();
        motorVelocitySignal = motor.getVelocity(false);
        motorSetpointSignal = motor.getClosedLoopReference(false);
    }

    public abstract Slot0Configs pidGains();

    public abstract AngularVelocity velocityTolerance();

    public abstract SysIdRoutine sysIdConfig();
}
