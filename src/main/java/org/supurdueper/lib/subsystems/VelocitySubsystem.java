package org.supurdueper.lib.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
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
    private final LoggedTunableNumber ki;
    private final LoggedTunableNumber kd;
    private final LoggedTunableNumber ks;
    private final LoggedTunableNumber kv;
    private final LoggedTunableNumber ka;
    private final LoggedTunableNumber profileKv;
    private final LoggedTunableNumber profileKa;
    private final LoggedTunableNumber profileV;
    private final LoggedTunableNumber profileA;
    private final List<LoggedTunableNumber> pidGains;
    private final MotionMagicVelocityTorqueCurrentFOC velocityRequest = new MotionMagicVelocityTorqueCurrentFOC(0);
    protected final AngularVelocity velocityTolerance;
    private final SysIdRoutine sysIdRoutine;
    private final Trigger atVelocity = new Trigger(this::atVelocity);

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
        return goToVelocity(angularvelocity).andThen(Commands.waitUntil(this::atVelocity));
    }

    public Trigger isAtVelocity() {
        return atVelocity;
    }

    protected void setVelocity(AngularVelocity angularvelocity) {
        motor.setControl(velocityRequest);
    }

    protected void setVelocity(double angularvelocity) {
        motor.setControl(velocityRequest);
    }

    protected AngularVelocity getPosition() {
        return motorVelocitySignal.getValue();
    }

    protected AngularVelocity getVelocity() {
        return Units.DegreesPerSecond.of(motorSetpointSignal.getValueAsDouble());
    }

    protected AngularVelocity getSetpoint() {
        return Units.DegreesPerSecond.of(motorSetpointSignal.getValueAsDouble());
    }

    protected boolean atVelocity() {
        return (getSetpoint().minus(getVelocity())).abs(RotationsPerSecond)
                < (velocityTolerance).abs(RotationsPerSecond);
    }

    public VelocitySubsystem() {
        super();

        // Setup tunable pid gains
        String name = this.getName();
        kp = new LoggedTunableNumber(name + "/Kp");
        ki = new LoggedTunableNumber(name + "/Ki");
        kd = new LoggedTunableNumber(name + "/Kd");
        ks = new LoggedTunableNumber(name + "/Ks");
        kv = new LoggedTunableNumber(name + "/Kv");
        ka = new LoggedTunableNumber(name + "/Ka");
        new LoggedTunableNumber(name + "/kfeedforward");
        profileKv = new LoggedTunableNumber(name + "/profileKv");
        profileKa = new LoggedTunableNumber(name + "/profileKa");
        profileV = new LoggedTunableNumber(name + "/profileVel");
        profileA = new LoggedTunableNumber(name + "/profileAcc");
        Slot0Configs gains = pidGains();
        kp.initDefault(gains.kP);
        ki.initDefault(gains.kI);
        kd.initDefault(gains.kD);
        ks.initDefault(gains.kS);
        kv.initDefault(gains.kV);
        ka.initDefault(gains.kA);
        MotionMagicConfigs motionMagicConfig = motionMagicConfig();
        profileKv.initDefault(motionMagicConfig.MotionMagicExpo_kV);
        profileKa.initDefault(motionMagicConfig.MotionMagicExpo_kA);
        profileV.initDefault(motionMagicConfig.MotionMagicCruiseVelocity);
        profileA.initDefault(motionMagicConfig.MotionMagicAcceleration);
        pidGains = new ArrayList<>();
        pidGains.addAll(List.of(kp, ki, kd, ks, kv, ka, profileKa, profileKv, profileV, profileA));
        velocityTolerance = velocityTolerance();
        sysIdRoutine = sysIdConfig();
        // Add motion magic items to config
        config = config.withSlot0(gains).withMotionMagic(motionMagicConfig).withSoftwareLimitSwitch(softLimitConfig());
    }

    @Override
    public void periodic() {
        if (Constants.tuningMode) {
            for (LoggedTunableNumber gain : pidGains) {
                if (gain.hasChanged(hashCode())) {
                    // Send new PID gains to talon
                    Slot0Configs slot0config = new Slot0Configs()
                            .withKP(kp.get())
                            .withKI(ki.get())
                            .withKD(kd.get())
                            .withKS(ks.get())
                            .withKV(kv.get())
                            .withKA(ka.get());
                    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                            .withMotionMagicExpo_kA(profileKa.get())
                            .withMotionMagicExpo_kV(profileKv.get())
                            .withMotionMagicCruiseVelocity(profileV.get())
                            .withMotionMagicAcceleration(profileA.get());
                    motor.getConfigurator().apply(config.withSlot0(slot0config).withMotionMagic(motionMagicConfigs));
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

    public abstract MotionMagicConfigs motionMagicConfig();

    public abstract SoftwareLimitSwitchConfigs softLimitConfig();

    public abstract AngularVelocity velocityTolerance();

    public abstract SysIdRoutine sysIdConfig();
}
