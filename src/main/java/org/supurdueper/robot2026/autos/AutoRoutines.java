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

    private final String leftOneRun = "Left_One_Run";
    private final String rightOneRun = "Right_One_Run";
    private final String leftTwoRun = "Left_Two_Run";
    private final String rightTwoRun = "Right_Two_Run";
    private final String rightFullRun = "Right_Full_Run";
    private final String leftFullRun = "Left_Full_Run";

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
        drivetrain = RobotContainer.getDrivetrain();
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

        return routine;
    }

    public AutoRoutine twoRun(
            AutoRoutine routine,
            AutoTrajectory overBump1,
            AutoTrajectory intakeBalls1,
            AutoTrajectory toHub1,
            AutoTrajectory overBump2,
            AutoTrajectory intakeBalls2,
            AutoTrajectory toHub2) {
        routine.active()
                .onTrue(overBump1
                        .resetOdometry()
                        .andThen(() -> RobotStates.setAutoDropIntake(true))
                        .andThen(overBump1.cmd())
                        .andThen(() -> RobotStates.setAutoDropIntake(false)));

        overBump1.chain(intakeBalls1);

        intakeBalls1
                .active()
                .whileTrue(Commands.runEnd(
                        (() -> RobotStates.setAutoIntake(true)), (() -> RobotStates.setAutoIntake(false))));

        intakeBalls1.chain(toHub1);

        toHub1.recentlyDone()
                .onTrue(Commands.sequence(
                        Commands.runOnce(() -> RobotStates.setAutoAim(true)),
                        Commands.runOnce(() -> RobotStates.setAutoShoot(true)),
                        Commands.waitSeconds(5),
                        Commands.runOnce(() -> {
                            RobotStates.setAutoAim(false);
                            RobotStates.setAutoShoot(false);
                        }),
                        overBump2.cmd()));

        overBump2.chain(intakeBalls2);

        intakeBalls2
                .active()
                .whileTrue(Commands.runEnd(
                        (() -> RobotStates.setAutoIntake(true)), (() -> RobotStates.setAutoIntake(false))));

        intakeBalls2.chain(toHub2);

        toHub2.recentlyDone()
                .onTrue(Commands.sequence(
                        Commands.runOnce(() -> RobotStates.setAutoAim(true)),
                        Commands.runOnce(() -> RobotStates.setAutoShoot(true))));

        return routine;
    }

    public AutoRoutine fullRun(
            AutoRoutine routine,
            AutoTrajectory overBumpOne,
            AutoTrajectory intakeBallsOne,
            AutoTrajectory toHubOne,
            AutoTrajectory overBumpTwo,
            AutoTrajectory intakeBallsTwo,
            AutoTrajectory toHubTwo) {
        routine.active()
                .onTrue(overBumpOne
                        .resetOdometry()
                        .andThen(() -> RobotStates.setAutoDropIntake(true))
                        .andThen(overBumpOne.cmd()));

        overBumpOne.chain(intakeBallsOne);

        intakeBallsOne
                .active()
                .whileTrue(Commands.runEnd(
                        (() -> RobotStates.setAutoIntake(true)), (() -> RobotStates.setAutoIntake(false))));

        intakeBallsOne.chain(toHubOne);

        toHubOne.recentlyDone()
                .onTrue(Commands.sequence(
                        Commands.runOnce(() -> RobotStates.setAutoAim(true)),
                        Commands.waitSeconds(0.5),
                        Commands.runOnce(() -> RobotStates.setAutoShoot(true)),
                        Commands.waitSeconds(5),
                        Commands.runOnce(() -> {
                            RobotStates.setAutoAim(false);
                            RobotStates.setAutoShoot(false);
                        }),
                        overBumpTwo.cmd().asProxy()));
        overBumpTwo.chain(intakeBallsTwo);

        intakeBallsTwo
                .active()
                .whileTrue(Commands.runEnd(
                        (() -> RobotStates.setAutoIntake(true)), (() -> RobotStates.setAutoIntake(false))));

        intakeBallsTwo.chain(toHubTwo);

        toHubTwo.recentlyDone()
                .onTrue(Commands.sequence(
                        (Commands.runOnce(() -> RobotStates.setAutoAim(true))),
                        (Commands.runOnce(() -> RobotStates.setAutoShoot(true)))));

        return routine;
    }

    public AutoRoutine leftOneRun() {
        AutoRoutine routine = m_factory.newRoutine("Left One Run");
        final AutoTrajectory leftOverBump = routine.trajectory(leftOneRun, 0);
        final AutoTrajectory leftIntakeBalls = routine.trajectory(leftOneRun, 1);
        final AutoTrajectory leftToHub = routine.trajectory(leftOneRun, 2);
        return oneRun(routine, leftOverBump, leftIntakeBalls, leftToHub);
    }

    public AutoRoutine rightOneRun() {
        AutoRoutine routine = m_factory.newRoutine("Right One Run");
        final AutoTrajectory rightOverBump = routine.trajectory(rightOneRun, 0);
        final AutoTrajectory rightIntakeBalls = routine.trajectory(rightOneRun, 1);
        final AutoTrajectory rightToHub = routine.trajectory(rightOneRun, 2);
        return oneRun(routine, rightOverBump, rightIntakeBalls, rightToHub);
    }

    public AutoRoutine rightTwoRun() {
        AutoRoutine routine = m_factory.newRoutine("Right Two Run");
        final AutoTrajectory rightOverBump1 = routine.trajectory(rightOneRun, 0);
        final AutoTrajectory rightIntakeBalls1 = routine.trajectory(rightOneRun, 1);
        final AutoTrajectory rightToHub1 = routine.trajectory(rightOneRun, 2);
        final AutoTrajectory rightOverBump2 = routine.trajectory(rightTwoRun, 0);
        final AutoTrajectory rightIntakeBalls2 = routine.trajectory(rightTwoRun, 1);
        final AutoTrajectory rightToHub2 = routine.trajectory(rightTwoRun, 2);
        return twoRun(
                routine,
                rightOverBump1,
                rightIntakeBalls1,
                rightToHub1,
                rightOverBump2,
                rightIntakeBalls2,
                rightToHub2);
    }

    public AutoRoutine leftTwoRun() {
        AutoRoutine routine = m_factory.newRoutine("Left Two Run");
        final AutoTrajectory leftOverBump1 = routine.trajectory(leftOneRun, 0);
        final AutoTrajectory leftIntakeBalls1 = routine.trajectory(leftOneRun, 1);
        final AutoTrajectory leftToHub1 = routine.trajectory(leftOneRun, 2);
        final AutoTrajectory leftOverBump2 = routine.trajectory(leftTwoRun, 0);
        final AutoTrajectory leftIntakeBalls2 = routine.trajectory(leftTwoRun, 1);
        final AutoTrajectory leftToHub2 = routine.trajectory(leftTwoRun, 2);
        return twoRun(
                routine, leftOverBump1, leftIntakeBalls1, leftToHub1, leftOverBump2, leftIntakeBalls2, leftToHub2);
    }

    public AutoRoutine rightFullRun() {
        AutoRoutine routine = m_factory.newRoutine("Right Full Run");
        final AutoTrajectory rightOverBumpOne = routine.trajectory("Right_Full_Run", 0);
        final AutoTrajectory rightIntakeBallsOne = routine.trajectory("Right_Full_Run", 1);
        final AutoTrajectory rightToHubOne = routine.trajectory("Right_Full_Run", 2);
        final AutoTrajectory rightOverBumpTwo = routine.trajectory("Right_Full_Run", 3);
        final AutoTrajectory rightIntakeBallsTwo = routine.trajectory("Right_Full_Run", 4);
        final AutoTrajectory rightToHubTwo = routine.trajectory("Right_Full_Run", 5);
        return fullRun(
                routine,
                rightOverBumpOne,
                rightIntakeBallsOne,
                rightToHubOne,
                rightOverBumpTwo,
                rightIntakeBallsTwo,
                rightToHubTwo);
    }

    public AutoRoutine leftFullRun() {
        AutoRoutine routine = m_factory.newRoutine("Left Full Run");
        final AutoTrajectory leftOverBumpOne = routine.trajectory(leftFullRun, 0);
        final AutoTrajectory leftIntakeBallsOne = routine.trajectory(leftFullRun, 1);
        final AutoTrajectory leftToHubOne = routine.trajectory(leftFullRun, 2);
        final AutoTrajectory leftOverBumpTwo = routine.trajectory(leftFullRun, 3);
        final AutoTrajectory leftIntakeBallsTwo = routine.trajectory(leftFullRun, 4);
        final AutoTrajectory leftToHubTwo = routine.trajectory(leftFullRun, 5);
        return fullRun(
                routine,
                leftOverBumpOne,
                leftIntakeBallsOne,
                leftToHubOne,
                leftOverBumpTwo,
                leftIntakeBallsTwo,
                leftToHubTwo);
    }
}
