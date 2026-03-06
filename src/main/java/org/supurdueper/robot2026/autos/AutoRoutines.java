package org.supurdueper.robot2026.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import org.supurdueper.robot2026.RobotContainer;
import org.supurdueper.robot2026.state.RobotStates;
import org.supurdueper.robot2026.subsystems.drive.Drivetrain;

public class AutoRoutines {
    private final AutoFactory m_factory;
    private final Drivetrain drivetrain;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
        drivetrain = RobotContainer.getDrivetrain();
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory simplePath = routine.trajectory("NewPath");

        routine.active().onTrue(simplePath.resetOdometry().andThen(simplePath.cmd()));
        return routine;
    }

    public AutoRoutine oneRun(
            AutoRoutine routine, AutoTrajectory overBump, AutoTrajectory intakeBalls, AutoTrajectory toHub) {

        routine.active()
                .onTrue(overBump.resetOdometry()
                        .andThen(() -> RobotStates.setAutoDropIntake(true))
                        .andThen(overBump.cmd()));

        overBump.chain(intakeBalls);

        intakeBalls
                .active()
                .whileTrue(Commands.runEnd(
                        (() -> RobotStates.setAutoIntake(true)), (() -> RobotStates.setAutoIntake(false))));

        intakeBalls.chain(toHub);

        toHub.recentlyDone()
                .onTrue(Commands.sequence(
                        (Commands.runOnce(() -> RobotStates.setAutoAim(true))),
                        (Commands.runOnce(() -> RobotStates.setAutoShoot(true)))));
        //     .withTimeout(5),
        // Commands.runOnce(() -> RobotStates.setAutoAim(false)),
        // Commands.runOnce(() -> RobotStates.setAutoShoot(false)),
        // Commands.runOnce(() -> RobotStates.setAutoDropIntake(false))));

        return routine;
    }

    public AutoRoutine leftOneRun() {
        AutoRoutine routine = m_factory.newRoutine("Left One Run");
        final AutoTrajectory leftOverBump = routine.trajectory("Left_One_Run", 0);
        final AutoTrajectory leftIntakeBalls = routine.trajectory("Left_One_Run", 1);
        final AutoTrajectory leftToHub = routine.trajectory("Left_One_Run", 2);
        return oneRun(routine, leftOverBump, leftIntakeBalls, leftToHub);
    }

    public AutoRoutine rightOneRun() {
        AutoRoutine routine = m_factory.newRoutine("Right One Run");
        final AutoTrajectory rightOverBump = routine.trajectory("Right_One_Run", 0);
        final AutoTrajectory rightIntakeBalls = routine.trajectory("Right_One_Run", 1);
        final AutoTrajectory rightToHub = routine.trajectory("Right_One_Run", 2);
        return oneRun(routine, rightOverBump, rightIntakeBalls, rightToHub);
    }
}
