// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026;

import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;
import org.supurdueper.robot2026.state.Driver;
import org.supurdueper.robot2026.state.DriverSticks;
import org.supurdueper.robot2026.state.RobotStates;
import org.supurdueper.robot2026.subsystems.Climber;
import org.supurdueper.robot2026.subsystems.Feeder;
import org.supurdueper.robot2026.subsystems.Hopper;
import org.supurdueper.robot2026.subsystems.Intake;
import org.supurdueper.robot2026.subsystems.Lights;
import org.supurdueper.robot2026.subsystems.Shooter;
import org.supurdueper.robot2026.subsystems.ShooterHood;
import org.supurdueper.robot2026.subsystems.Uptake;
import org.supurdueper.robot2026.subsystems.Vision;
import org.supurdueper.robot2026.subsystems.drive.Drivetrain;
import org.supurdueper.robot2026.subsystems.drive.generated.TunerConstants;
import org.supurdueper.robot2026.utils.HubShiftUtil;

public class RobotContainer {

    @Getter
    private static Drivetrain drivetrain;

    @Getter
    private static DriverSticks driver;

    @Getter
    private static Driver testController;

    @Getter
    private static Shooter shooter;

    @Getter
    private static Feeder feeder;

    @Getter
    private static Hopper hopper;

    @Getter
    private static Intake intake;

    @Getter
    private static Climber climber;

    @Getter
    private static ShooterHood shooterHood;

    @Getter
    private static Vision vision;

    @Getter
    private static Lights lights;

    @Getter
    private Uptake uptake;

    public RobotContainer() {
        driver = new DriverSticks();
        testController = new Driver(Constants.DriverConstants.kTestControllerPort);
        drivetrain = TunerConstants.createDrivetrain();
        intake = new Intake();
        hopper = new Hopper();
        feeder = new Feeder();
        shooter = new Shooter();
        uptake = new Uptake();
        shooterHood = new ShooterHood();
        vision = new Vision();
        // lights = new Lights();
        // climber = new Climber();
        configureBindings();
    }

    public void configureBindings() {
        // Reset hub shift timer when enabling
        RobotStates.teleop.onTrue(Commands.runOnce(HubShiftUtil::initialize));
        RobotStates.auto.onTrue(Commands.runOnce(HubShiftUtil::initialize));
        RobotStates.disabled.onTrue(Commands.runOnce(HubShiftUtil::initialize).ignoringDisable(true));
    }
}
