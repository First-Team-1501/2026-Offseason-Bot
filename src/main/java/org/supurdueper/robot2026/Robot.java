// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import com.ctre.phoenix6.HootAutoReplay;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.supurdueper.BuildConstants;
import org.supurdueper.lib.LoggedTunableNumber;
import org.supurdueper.lib.subsystems.SupurdueperRobot;
import org.supurdueper.robot2026.autos.AutoRoutines;
import org.supurdueper.robot2026.state.RobotStates;
import org.supurdueper.robot2026.subsystems.Vision;
import org.supurdueper.robot2026.utils.FieldConstants;

public class Robot extends SupurdueperRobot {
    @SuppressWarnings("unused")
    private final RobotContainer m_robotContainer;

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();
    private final LoggedTunableNumber autoTimeout;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay =
            new HootAutoReplay().withTimestampReplay().withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        autoFactory = RobotContainer.getDrivetrain().createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);
        autoTimeout = new LoggedTunableNumber("Auto Timeout", 1.0);

        autoChooser.addRoutine("Left 1 Run", autoRoutines::leftOneRun);
        autoChooser.addRoutine("Right 1 Run", autoRoutines::rightOneRun);
        autoChooser.addRoutine("Right Full Run", autoRoutines::rightFullRun);
        autoChooser.addRoutine("Left Full Run", autoRoutines::leftFullRun);
        autoChooser.addRoutine("Left Wall Run", autoRoutines::leftWallRun);
        autoChooser.addRoutine("Right Wall Run", autoRoutines::rightWallRun);
        autoChooser.addRoutine("Right ONLY Wall Run", autoRoutines::rightOnlyWall);
        autoChooser.addRoutine("Left ONLY Wall Run", autoRoutines::leftOnlyWall);

        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putNumber("Auto Timeout", autoTimeout.get());
    }

    @Override
    public void robotInit() {
        DogLog.setOptions(new DogLogOptions()
                .withLogExtras(false)
                .withCaptureDs(false)
                .withNtPublish(Constants.publishToNT)
                .withCaptureNt(false));
        // Record metadata
        DogLog.log("Git/ProjectName", BuildConstants.MAVEN_NAME);
        DogLog.log("Git/BuildDate", BuildConstants.BUILD_DATE);
        DogLog.log("Git/GitSHA", BuildConstants.GIT_SHA);
        DogLog.log("Git/GitDate", BuildConstants.GIT_DATE);
        DogLog.log("Git/GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                DogLog.log("Git/GitDirty", "All changes committed");
                break;
            case 1:
                DogLog.log("Git/GitDirty", "Uncomitted changes");
                break;
            default:
                DogLog.log("Git/GitDirty", "Unknown");
                break;
        }

        // Limelight port fowarding
        // for (int port = 5800; port <= 5809; port++) {
        //     PortForwarder.add(port, "10.74.57.200", port);
        //     PortForwarder.add(port + 100, "10.74.57.201", port);
        // }

        FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31);
        resetCommandsAndButtons();

        if (!DriverStation.isFMSAttached()) SmartDashboard.putBoolean("Won Auto", true);
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        Threads.setCurrentThreadPriority(true, 1);
        double startTime = Timer.getFPGATimestamp();
        CommandScheduler.getInstance().run();
        double endTime = Timer.getFPGATimestamp();
        Threads.setCurrentThreadPriority(false, 0);
        DogLog.log("Loop Time", endTime - startTime);
        // for bunker hub lights to get match time from network tables
        RobotStates.log();
        if (!DriverStation.isFMSAttached()) {
            SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
        } else {
            DogLog.log("Match Time", Timer.getMatchTime());
        }
    }

    @Override
    public void disabledInit() {
        Vision.setDisabled();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        Vision.setEnabled();
        Vision.setAprilTagFilter();
        Vision.updateIMUMode();
        autoChooser.selectedCommandScheduler().schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        resetCommandsAndButtons();
        Vision.setEnabled();
        Vision.setAprilTagFilter();
        Vision.updateIMUMode();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        resetCommandsAndButtons();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}

    /**
     * This method cancels all commands and returns subsystems to their default commands and the gamepad configs are
     * reset so that new bindings can be assigned based on mode This method should be called when each mode is
     * initialized
     */
    public void resetCommandsAndButtons() {
        CommandScheduler.getInstance().cancelAll(); // Disable any currently running commands
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        // Bind Triggers for all subsystems
        bindCommands();
    }
}
