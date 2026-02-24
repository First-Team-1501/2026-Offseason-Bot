package org.supurdueper.robot2026.state;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.supurdueper.robot2026.RobotContainer;

public final class RobotStates {

    public static final Trigger sim = new Trigger(RobotBase::isSimulation);
    public static final Trigger teleop = RobotModeTriggers.teleop();
    public static final Trigger auto = RobotModeTriggers.autonomous();
    public static final Trigger disabled = RobotModeTriggers.disabled();
    public static final Driver driver = RobotContainer.getDriver();
    public static final Driver testController = RobotContainer.getTestController();

    // auto

    // information

    // Actions
    public static final Trigger rezeroFieldHeading = driver.select.and(teleop);
    public static final Trigger actionIntake = driver.A.and(teleop);
    public static final Trigger actionPurge = driver.extraRight.and(teleop);
    public static final Trigger actionShoot = driver.leftBumper.and(teleop);
    public static final Trigger actionClimbPrep = driver.downDpad.and(teleop);
    public static final Trigger actionClimb = driver.leftDpad.and(teleop);

    public static final Trigger actionTestIntake = testController.A.and(teleop);
    public static final Trigger actionTestHopper = testController.B.and(teleop);
    public static final Trigger actionTestFeeder = testController.X.and(teleop);
    public static final Trigger actionTestShooter = testController.Y.and(teleop);

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }

    public static void setAimed(boolean b) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setAimed'");
    }
}
