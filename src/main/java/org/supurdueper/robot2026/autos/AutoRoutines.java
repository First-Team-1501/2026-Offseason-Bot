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

    private final String rightFullRun = "Right_Full_Run";
    private final String leftFullRun = "Left_Full_Run";
    private final String leftWallRun = "Left_Wall_Run";
    private final String rightWallRun = "Right_Wall_Run";

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
        drivetrain = RobotContainer.getDrivetrain();
    }

    public AutoRoutine oneRun(
            AutoRoutine routine, AutoTrajectory overBump, AutoTrajectory intakeBalls, AutoTrajectory toHub) {

        routine.active()
                .onTrue(overBump.resetOdometry()
                        .andThen(() -> RobotStates.setAutoDropIntake(true))
                        .andThen(() -> RobotStates.setAutoIntake(true))
                        .andThen(overBump.cmd()));

        overBump.chain(intakeBalls);

        intakeBalls.chain(toHub);

        toHub.recentlyDone()
                .onTrue(Commands.sequence(
                        (Commands.runOnce(() -> RobotStates.setAutoAim(true))),
                        (Commands.runOnce(() -> RobotStates.setAutoShoot(true))),
                        (Commands.runOnce(() -> RobotStates.setAutoIntake(false)))));

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
                        .andThen(() -> RobotStates.setAutoIntake(true))
                        .andThen(overBumpOne.cmd()));

        overBumpOne.chain(intakeBallsOne);

        intakeBallsOne.chain(toHubOne);

        toHubOne.recentlyDone()
                .onTrue(Commands.sequence(
                        Commands.runOnce(() -> RobotStates.setAutoAim(true)),
                        Commands.waitSeconds(0.5),
                        Commands.runOnce(() -> RobotStates.setAutoShoot(true)),
                        Commands.waitSeconds(3),
                        Commands.runOnce(() -> {
                            RobotStates.setAutoAim(false);
                            RobotStates.setAutoShoot(false);
                        }),
                        overBumpTwo.cmd().asProxy()));
        overBumpTwo.chain(intakeBallsTwo);

        intakeBallsTwo.chain(toHubTwo);

        toHubTwo.recentlyDone()
                .onTrue(Commands.sequence(
                        (Commands.runOnce(() -> RobotStates.setAutoAim(true))),
                        (Commands.runOnce(() -> RobotStates.setAutoShoot(true))),
                        (Commands.runOnce(() -> RobotStates.setAutoIntake(true)))));

        return routine;
    }

    public AutoRoutine leftOneRun() {
        AutoRoutine routine = m_factory.newRoutine("Left One Run");
        final AutoTrajectory leftOverBump = routine.trajectory(leftFullRun, 0);
        final AutoTrajectory leftIntakeBalls = routine.trajectory(leftFullRun, 1);
        final AutoTrajectory leftToHub = routine.trajectory(leftFullRun, 2);
        return oneRun(routine, leftOverBump, leftIntakeBalls, leftToHub);
    }

    public AutoRoutine rightOneRun() {
        AutoRoutine routine = m_factory.newRoutine("Right One Run");
        final AutoTrajectory rightOverBump = routine.trajectory(rightFullRun, 0);
        final AutoTrajectory rightIntakeBalls = routine.trajectory(rightFullRun, 1);
        final AutoTrajectory rightToHub = routine.trajectory(rightFullRun, 2);
        return oneRun(routine, rightOverBump, rightIntakeBalls, rightToHub);
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

    public AutoRoutine leftWallRun() {
        AutoRoutine routine = m_factory.newRoutine("Left Wall Run");
        final AutoTrajectory leftOverBumpOne = routine.trajectory(leftWallRun, 0);
        final AutoTrajectory leftIntakeBallsOne = routine.trajectory(leftWallRun, 1);
        final AutoTrajectory leftToHubOne = routine.trajectory(leftWallRun, 2);
        final AutoTrajectory leftOverBumpTwo = routine.trajectory(leftWallRun, 3);
        final AutoTrajectory leftIntakeBallsTwo = routine.trajectory(leftWallRun, 4);
        final AutoTrajectory leftToHubTwo = routine.trajectory(leftWallRun, 5);
        return fullRun(
                routine,
                leftOverBumpOne,
                leftIntakeBallsOne,
                leftToHubOne,
                leftOverBumpTwo,
                leftIntakeBallsTwo,
                leftToHubTwo);
    }

    public AutoRoutine rightWallRun() {
        AutoRoutine routine = m_factory.newRoutine("Right Wall Run");
        final AutoTrajectory rightOverBumpOne = routine.trajectory(rightWallRun, 0);
        final AutoTrajectory rightIntakeBallsOne = routine.trajectory(rightWallRun, 1);
        final AutoTrajectory rightToHubOne = routine.trajectory(rightWallRun, 2);
        final AutoTrajectory rightOverBumpTwo = routine.trajectory(rightWallRun, 3);
        final AutoTrajectory rightIntakeBallsTwo = routine.trajectory(rightWallRun, 4);
        final AutoTrajectory rightToHubTwo = routine.trajectory(rightWallRun, 5);
        return fullRun(
                routine,
                rightOverBumpOne,
                rightIntakeBallsOne,
                rightToHubOne,
                rightOverBumpTwo,
                rightIntakeBallsTwo,
                rightToHubTwo);
    }
}
