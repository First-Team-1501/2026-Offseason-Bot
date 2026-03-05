package org.supurdueper.robot2026.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.supurdueper.robot2026.RobotContainer;
import org.supurdueper.robot2026.state.RobotStates;
import org.supurdueper.robot2026.subsystems.Intake;
import org.supurdueper.robot2026.subsystems.drive.Drivetrain;

public class AutoRoutines {
    private final AutoFactory m_factory;
    private final Drivetrain drivetrain;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
        drivetrain = RobotContainer.getDrivetrain();
    }

    public static Command dropIntake() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotContainer.getClimber().releaseIntake()));
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory simplePath = routine.trajectory("NewPath");

        routine.active().onTrue(simplePath.resetOdometry().andThen(simplePath.cmd()));
        return routine;
    }

    public AutoRoutine leftOneRun() {
        AutoRoutine routine = m_factory.newRoutine("Left One Run");
        final AutoTrajectory leftOverBump = routine.trajectory("Left_One_Run, 0");
        final AutoTrajectory leftIntakeBalls = routine.trajectory("Left_One_Run, 1");
        final AutoTrajectory leftToHub = routine.trajectory("Left_One_Run, 2");

        routine.active()
                .onTrue(leftOverBump.resetOdometry().andThen(dropIntake()).andThen(leftOverBump.cmd()));

        leftOverBump.recentlyDone().onTrue(leftIntakeBalls.cmd());

        leftIntakeBalls
                .active()
                .whileTrue(Commands.runEnd(
                        (() -> RobotStates.setAutoIntake(true)), (() -> RobotStates.setAutoIntake(false))));
        leftIntakeBalls.recentlyDone().onTrue(leftToHub.cmd());

        leftToHub
                .recentlyDone()
                .onTrue(Commands.sequence(
                        (Commands.runOnce(() -> RobotStates.setAutoAim(true))),
                        Commands.waitUntil(RobotStates.infoReadyToShoot)
                                .andThen(Commands.runOnce(() -> RobotStates.setAutoShoot(true)))
                                .withTimeout(5),
                        Commands.runOnce(() -> RobotStates.setAutoAim(false)),
                        Commands.runOnce(() -> RobotStates.setAutoShoot(false))));

        return routine;
    }

    public AutoRoutine rightOneRun() {
        AutoRoutine routine = m_factory.newRoutine("Right One Run");
        final AutoTrajectory rightOverBump = routine.trajectory("Right_One_Run, 0");
        final AutoTrajectory rightIntakeBalls = routine.trajectory("Right_One_Run, 1");
        final AutoTrajectory rightToHub = routine.trajectory("Right_One_Run, 2");

        routine.active()
                .onTrue(rightOverBump.resetOdometry().andThen(dropIntake()).andThen(rightOverBump.cmd()));

        rightOverBump.recentlyDone().onTrue(rightIntakeBalls.cmd());

        rightIntakeBalls
                .active()
                .whileTrue(Commands.runEnd(
                        (() -> RobotStates.setAutoIntake(true)), (() -> RobotStates.setAutoIntake(false))));
        rightIntakeBalls.recentlyDone().onTrue(rightToHub.cmd());

        rightToHub
                .recentlyDone()
                .onTrue(Commands.sequence(
                        (Commands.runOnce(() -> RobotStates.setAutoAim(true))),
                        Commands.waitUntil(RobotStates.infoReadyToShoot)
                                .andThen(Commands.runOnce(() -> RobotStates.setAutoShoot(true)))
                                .withTimeout(5),
                        Commands.runOnce(() -> RobotStates.setAutoAim(false)),
                        Commands.runOnce(() -> RobotStates.setAutoShoot(false))));

        return routine;
    }
}
