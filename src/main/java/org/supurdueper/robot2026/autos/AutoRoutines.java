package org.supurdueper.robot2026.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final String rightOnlyWall = "Right_Only_Wall";
    private final String leftOnlyWall = "Left_Only_Wall";
    private final String rightTrenchStart = "Trench_Right_Start";
    private final String rightTrenchTwo = "Trench_Right_Second";
    private final String leftTrenchStart = "Trench_Left_Start";
    private final String leftTrenchTwo = "Trench_Left_Second";
    private final String leftTrenchShort = "Trench_Left_Second_Short";
    private final String rightTrenchShort = "Trench_Right_Short";
    private final String leftTrenchThird = "Trench_Left_Third";
    private final String rightTrenchThird = "Trench_Right_Third";

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
        drivetrain = RobotContainer.getDrivetrain();
    }

    public AutoRoutine oneRun(
            AutoRoutine routine, AutoTrajectory overBump, AutoTrajectory intakeBalls, AutoTrajectory toHub) {

        routine.active()
                .onTrue(overBump.resetOdometry()
                        .andThen(() -> RobotStates.setAutoDropIntake(true))
                        .andThen(Commands.waitSeconds(SmartDashboard.getNumber("Auto Timeout", 0)))
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
                        .andThen(Commands.waitSeconds(SmartDashboard.getNumber("Auto Timeout", 0)))
                        .andThen(() -> RobotStates.setAutoIntake(true))
                        .andThen(overBumpOne.cmd()));

        overBumpOne.chain(intakeBallsOne);

        intakeBallsOne.chain(toHubOne);

        toHubOne.recentlyDone()
                .onTrue(Commands.sequence(
                        Commands.runOnce(() -> RobotStates.setAutoIntake(false)),
                        Commands.runOnce(() -> RobotStates.setAutoAim(true)),
                        Commands.waitSeconds(0.5),
                        Commands.runOnce(() -> RobotStates.setAutoShoot(true)),
                        Commands.waitSeconds(3),
                        Commands.runOnce(() -> {
                            RobotStates.setAutoAim(false);
                            RobotStates.setAutoShoot(false);
                        }),
                        Commands.runOnce(() -> RobotStates.setAutoIntake(true)),
                        overBumpTwo.cmd().asProxy()));
        overBumpTwo.chain(intakeBallsTwo);

        intakeBallsTwo.chain(toHubTwo);

        toHubTwo.recentlyDone()
                .onTrue(Commands.sequence(
                        (Commands.runOnce(() -> RobotStates.setAutoIntake(false))),
                        (Commands.runOnce(() -> RobotStates.setAutoAim(true))),
                        (Commands.waitSeconds(0.5)),
                        (Commands.runOnce(() -> RobotStates.setAutoShoot(true))),
                        (Commands.runOnce(() -> RobotStates.setAutoIntake(true)))));

        return routine;
    }

    public AutoRoutine trenchRun(
            AutoRoutine routine,
            AutoTrajectory underTrenchOne,
            AutoTrajectory toCenterOne,
            AutoTrajectory toScoreOne,
            AutoTrajectory underTrenchTwo,
            AutoTrajectory toCenterTwo,
            AutoTrajectory toScoreTwo,
            AutoTrajectory toCenterThree) {
        routine.active()
                .onTrue(underTrenchOne
                        .resetOdometry()
                        .andThen(Commands.waitSeconds(SmartDashboard.getNumber("Auto Timeout", 0)))
                        .andThen(underTrenchOne.cmd()));

        underTrenchOne
                .recentlyDone()
                .onTrue(Commands.runOnce(() -> RobotStates.setAutoDropIntake(true))
                        .andThen(Commands.runOnce(() -> RobotStates.setAutoIntake(true))));

        underTrenchOne.chain(toCenterOne);

        toCenterOne.chain(toScoreOne);

        toCenterOne.recentlyDone().onTrue(Commands.runOnce(() -> RobotStates.setAutoRev(true)));

        toScoreOne
                .recentlyDone()
                .onTrue(Commands.sequence(
                        Commands.runOnce(() -> RobotStates.setAutoIntake(false)),
                        Commands.runOnce(() -> RobotStates.setAutoRev(false)),
                        Commands.runOnce(() -> RobotStates.setAutoAim(true)),
                        Commands.waitSeconds(0.5),
                        Commands.runOnce(() -> RobotStates.setAutoShoot(true)),
                        Commands.waitUntil(() -> RobotContainer.getHopper().noFuel())
                                .withDeadline(Commands.waitSeconds(3)),
                        Commands.runOnce(() -> {
                            RobotStates.setAutoAim(false);
                            RobotStates.setAutoShoot(false);
                        }),
                        Commands.runOnce(() -> RobotStates.setAutoIntake(true)),
                        underTrenchTwo.cmd().asProxy()));

        toScoreOne.chain(underTrenchTwo);

        underTrenchTwo.chain(toCenterTwo);
        toCenterTwo.chain(toScoreTwo);

        toCenterTwo.recentlyDone().onTrue(Commands.runOnce(() -> RobotStates.setAutoRev(true)));

        toScoreTwo
                .recentlyDone()
                .onTrue(Commands.sequence(
                        Commands.runOnce(() -> RobotStates.setAutoIntake(false)),
                        Commands.runOnce(() -> RobotStates.setAutoRev(false)),
                        Commands.waitSeconds(0.5),
                        Commands.runOnce(() -> RobotStates.setAutoShoot(true)),
                        Commands.waitUntil(() -> RobotContainer.getHopper().noFuel())
                                .withDeadline(Commands.waitSeconds(3)),
                        Commands.runOnce(() -> {
                            RobotStates.setAutoAim(false);
                            RobotStates.setAutoShoot(false);
                        }),
                        Commands.runOnce(() -> RobotStates.setAutoIntake(true)),
                        toCenterThree.cmd().asProxy()));

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

    public AutoRoutine rightOnlyWall() {
        AutoRoutine routine = m_factory.newRoutine("Right_Only_Wall");
        final AutoTrajectory rightOnlyOverBumpOne = routine.trajectory(rightOnlyWall, 0);
        final AutoTrajectory rightOnlyIntakeBallsOne = routine.trajectory(rightOnlyWall, 1);
        final AutoTrajectory rightOnlyToHubOne = routine.trajectory(rightOnlyWall, 2);
        final AutoTrajectory rightOnlyOverBumpTwo = routine.trajectory(rightOnlyWall, 3);
        final AutoTrajectory rightOnlyIntakeBallsTwo = routine.trajectory(rightOnlyWall, 4);
        final AutoTrajectory rightOnlyToHubTwo = routine.trajectory(rightOnlyWall, 5);
        return fullRun(
                routine,
                rightOnlyOverBumpOne,
                rightOnlyIntakeBallsOne,
                rightOnlyToHubOne,
                rightOnlyOverBumpTwo,
                rightOnlyIntakeBallsTwo,
                rightOnlyToHubTwo);
    }

    public AutoRoutine leftOnlyWall() {
        AutoRoutine routine = m_factory.newRoutine("Left_Only_Wall");
        final AutoTrajectory leftOnlyOverBumpOne = routine.trajectory(leftOnlyWall, 0);
        final AutoTrajectory leftOnlyIntakeBallsOne = routine.trajectory(leftOnlyWall, 1);
        final AutoTrajectory leftOnlyToHubOne = routine.trajectory(leftOnlyWall, 2);
        final AutoTrajectory leftOnlyOverBumpTwo = routine.trajectory(leftOnlyWall, 3);
        final AutoTrajectory leftOnlyIntakeBallsTwo = routine.trajectory(leftOnlyWall, 4);
        final AutoTrajectory leftOnlyToHubTwo = routine.trajectory(leftOnlyWall, 5);
        return fullRun(
                routine,
                leftOnlyOverBumpOne,
                leftOnlyIntakeBallsOne,
                leftOnlyToHubOne,
                leftOnlyOverBumpTwo,
                leftOnlyIntakeBallsTwo,
                leftOnlyToHubTwo);
    }

    public AutoRoutine rightTrench() {
        AutoRoutine routine = m_factory.newRoutine("Right_Trench");
        final AutoTrajectory rightUnderTrenchOne = routine.trajectory(rightTrenchStart, 0);
        final AutoTrajectory rightToCenterOne = routine.trajectory(rightTrenchStart, 1);
        final AutoTrajectory rightToScoreOne = routine.trajectory(rightTrenchStart, 2);
        final AutoTrajectory rightUnderTrenchTwo = routine.trajectory(rightTrenchTwo, 0);
        final AutoTrajectory rightToCenterTwo = routine.trajectory(rightTrenchTwo, 1);
        final AutoTrajectory rightToScoreTwo = routine.trajectory(rightTrenchTwo, 2);
        final AutoTrajectory rightToCenterThree = routine.trajectory(rightTrenchThird, 0);
        return trenchRun(
                routine,
                rightUnderTrenchOne,
                rightToCenterOne,
                rightToScoreOne,
                rightUnderTrenchTwo,
                rightToCenterTwo,
                rightToScoreTwo,
                rightToCenterThree);
    }

    public AutoRoutine leftTrench() {
        AutoRoutine routine = m_factory.newRoutine("Left_Trench");
        final AutoTrajectory leftUnderTrenchOne = routine.trajectory(leftTrenchStart, 0);
        final AutoTrajectory leftToCenterOne = routine.trajectory(leftTrenchStart, 1);
        final AutoTrajectory leftToScoreOne = routine.trajectory(leftTrenchStart, 2);
        final AutoTrajectory leftUnderTrenchTwo = routine.trajectory(leftTrenchTwo, 0);
        final AutoTrajectory leftToCenterTwo = routine.trajectory(leftTrenchTwo, 1);
        final AutoTrajectory leftToScoreTwo = routine.trajectory(leftTrenchTwo, 2);
        final AutoTrajectory leftToCenterThree = routine.trajectory(leftTrenchThird, 0);
        return trenchRun(
                routine,
                leftUnderTrenchOne,
                leftToCenterOne,
                leftToScoreOne,
                leftUnderTrenchTwo,
                leftToCenterTwo,
                leftToScoreTwo,
                leftToCenterThree);
    }

    public AutoRoutine leftShortTrench() {
        AutoRoutine routine = m_factory.newRoutine("Left_Trench");
        final AutoTrajectory leftUnderTrenchOne = routine.trajectory(leftTrenchStart, 0);
        final AutoTrajectory leftToCenterOne = routine.trajectory(leftTrenchStart, 1);
        final AutoTrajectory leftToScoreOne = routine.trajectory(leftTrenchStart, 2);
        final AutoTrajectory leftUnderTrenchTwo = routine.trajectory(leftTrenchShort, 0);
        final AutoTrajectory leftToCenterTwo = routine.trajectory(leftTrenchShort, 1);
        final AutoTrajectory leftToScoreTwo = routine.trajectory(leftTrenchShort, 2);
        final AutoTrajectory leftToCenterThree = routine.trajectory(leftTrenchThird, 0);
        return trenchRun(
                routine,
                leftUnderTrenchOne,
                leftToCenterOne,
                leftToScoreOne,
                leftUnderTrenchTwo,
                leftToCenterTwo,
                leftToScoreTwo,
                leftToCenterThree);
    }

    public AutoRoutine rightTrenchShort() {
        AutoRoutine routine = m_factory.newRoutine("Right_Trench");
        final AutoTrajectory rightUnderTrenchOne = routine.trajectory(rightTrenchStart, 0);
        final AutoTrajectory rightToCenterOne = routine.trajectory(rightTrenchStart, 1);
        final AutoTrajectory rightToScoreOne = routine.trajectory(rightTrenchStart, 2);
        final AutoTrajectory rightUnderTrenchTwo = routine.trajectory(rightTrenchShort, 0);
        final AutoTrajectory rightToCenterTwo = routine.trajectory(rightTrenchShort, 1);
        final AutoTrajectory rightToScoreTwo = routine.trajectory(rightTrenchShort, 2);
        final AutoTrajectory rightToCenterThree = routine.trajectory(rightTrenchThird, 0);
        return trenchRun(
                routine,
                rightUnderTrenchOne,
                rightToCenterOne,
                rightToScoreOne,
                rightUnderTrenchTwo,
                rightToCenterTwo,
                rightToScoreTwo,
                rightToCenterThree);
    }
}
