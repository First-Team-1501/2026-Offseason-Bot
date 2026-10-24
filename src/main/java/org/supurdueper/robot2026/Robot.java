// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2026;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import com.ctre.phoenix6.SignalLogger;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.supurdueper.BuildConstants;
import org.supurdueper.lib.LoggedTunableNumber;
import org.supurdueper.lib.subsystems.SupurdueperRobot;
import org.supurdueper.lib.telemetry.TuningTelemetry;
import org.supurdueper.robot2026.autos.AutoRoutines;
import org.supurdueper.robot2026.state.RobotStates;
import org.supurdueper.robot2026.subsystems.Vision;
import org.supurdueper.robot2026.utils.FieldConstants;

public class Robot extends SupurdueperRobot {
    private final RobotContainer m_robotContainer;

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();
    private final LoggedTunableNumber autoTimeout;

    private static final String kBuildSummary = "Build: " + BuildConstants.MAVEN_NAME + " sha="
            + BuildConstants.GIT_SHA + " branch=" + BuildConstants.GIT_BRANCH + " dirty=" + BuildConstants.DIRTY
            + " built=" + BuildConstants.BUILD_DATE;
    private int loopCount = 0;

    public Robot() {
        // CTRE hoot logging off before any device exists: it starts with the CANivore devices and created a hoot folder
        // on every code start (the tuning data goes to the .wpilog under /Tuning)
        SignalLogger.enableAutoLogging(false);
        SignalLogger.stop();
        m_robotContainer = new RobotContainer();
        autoFactory = RobotContainer.getDrivetrain().createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);
        autoTimeout = new LoggedTunableNumber("Auto Timeout", 0.0);

        // autoChooser.addRoutine("Left 1 Run", autoRoutines::leftOneRun);
        // autoChooser.addRoutine("Right 1 Run", autoRoutines::rightOneRun);
        autoChooser.addRoutine("Right Full Run", autoRoutines::rightFullRun);
        autoChooser.addRoutine("Left Full Run", autoRoutines::leftFullRun);
        autoChooser.addRoutine("Left Wall Run", autoRoutines::leftWallRun);
        autoChooser.addRoutine("Right Wall Run", autoRoutines::rightWallRun);
        autoChooser.addRoutine("Right ONLY Wall Run", autoRoutines::rightOnlyWall);
        autoChooser.addRoutine("Left ONLY Wall Run", autoRoutines::leftOnlyWall);
        autoChooser.addRoutine("Trench Right Run", autoRoutines::rightTrench);
        autoChooser.addRoutine("Trench Left Run", autoRoutines::leftTrench);
        autoChooser.addRoutine("Trench Left Short", autoRoutines::leftShortTrench);
        autoChooser.addRoutine("Trench Right Short", autoRoutines::rightTrenchShort);
        // autoChooser.addRoutine("Right Trench Steal", autoRoutines::rightTrenchSteal);
        // autoChooser.addRoutine("Left Trench Steal", autoRoutines::leftTrenchSteal);
        autoChooser.addRoutine("Left Delay", autoRoutines::leftTrenchDelayShort);
        autoChooser.addRoutine("Right Delay", autoRoutines::rightTrenchDelayShort);

        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putNumber("Auto Timeout", autoTimeout.get());
    }

    @Override
    public void robotInit() {
        // Again after construction, in case a device started it
        SignalLogger.stop();
        DogLog.setOptions(new DogLogOptions()
                .withLogExtras(false)
                .withCaptureDs(false)
                .withNtPublish(Constants.publishToNT)
                .withCaptureNt(false)
                // Default 1000 overflowed at startup on the robot; per-motor shooter logging adds more
                .withLogEntryQueueCapacity(4000));
        Vision.setCameraPose();

        // Limelight port fowarding
        // for (int port = 5800; port <= 5809; port++) {
        //     PortForwarder.add(port, "10.74.57.200", port);
        //     PortForwarder.add(port + 100, "10.74.57.201", port);
        // }

        FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31);
        resetCommandsAndButtons();

        if (!DriverStation.isFMSAttached()) SmartDashboard.putBoolean("Won Auto", true);
        RobotController.setBrownoutVoltage(5.75);
        // Serve the deploy folder so Elastic's "Download From Robot" can load deploy/elastic-layout.json
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    }

    @Override
    public void robotPeriodic() {
        if (++loopCount == 50) {
            logBuildInfo();
        }
        // Real-time priority for the whole periodic: on the CPU-saturated rio the work after the scheduler (telemetry,
        // logging) was preempted at normal priority and overran the loop
        Threads.setCurrentThreadPriority(true, 1);
        double startTime = Timer.getFPGATimestamp();
        CommandScheduler.getInstance().run();
        double endTime = Timer.getFPGATimestamp();
        DogLog.log("Loop Time", endTime - startTime);
        TuningTelemetry.periodic();
        // for bunker hub lights to get match time from network tables
        RobotStates.log();
        if (!DriverStation.isFMSAttached()) {
            SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
        } else {
            DogLog.log("Match Time", Timer.getMatchTime());
        }
        Threads.setCurrentThreadPriority(false, 0);
    }

    /**
     * Record which build is running, about 1 s into the main loop. Robot construction takes over 10 s, so
     * DataLogManager has paused data records during robotInit; and DogLog never rewrites an unchanged string, so a
     * value dropped then is never written later.
     */
    private void logBuildInfo() {
        DataLogManager.log(kBuildSummary);
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
    }

    @Override
    public void disabledInit() {
        Vision.setCameraPose();
        Vision.setDisabled();
        // Also while disabled, so the MegaTag1 heading seed uses the filtered tags before the first enable
        Vision.setAprilTagFilter();
        Vision.updateIMUMode();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        DataLogManager.log(kBuildSummary);
        Vision.setEnabled();
        Vision.setAprilTagFilter();
        Vision.updateIMUMode();
        RobotStates.clearAutoFlags();
        RobotContainer.getShooterHood().zeroIfNeeded();
        autoChooser.selectedCommandScheduler().schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        DataLogManager.log(kBuildSummary);
        RobotStates.clearAutoFlags();
        resetCommandsAndButtons();
        Vision.setEnabled();
        Vision.setAprilTagFilter();
        Vision.updateIMUMode();
        RobotContainer.getShooterHood().zeroIfNeeded();
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
        // RobotContainer bindings (defense toggle, hub shift reset) live in the same loop and were lost here before
        m_robotContainer.configureBindings();
    }
}
