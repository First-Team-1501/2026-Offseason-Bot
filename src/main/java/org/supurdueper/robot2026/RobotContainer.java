// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026;

import static edu.wpi.first.units.Units.*;

import lombok.Getter;
import org.supurdueper.robot2026.state.Driver;
import org.supurdueper.robot2026.subsystems.drive.Drivetrain;
import org.supurdueper.robot2026.subsystems.drive.generated.Telemetry;
import org.supurdueper.robot2026.subsystems.drive.generated.TunerConstants;

public class RobotContainer {
    private double MaxSpeed =
            1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final Telemetry logger = new Telemetry(MaxSpeed);

    @Getter
    private static Drivetrain drivetrain;

    @Getter
    private static Driver driver;

    public RobotContainer() {
        drivetrain = TunerConstants.createDrivetrain();
        configureBindings();
    }

    public void configureBindings() {}
}
