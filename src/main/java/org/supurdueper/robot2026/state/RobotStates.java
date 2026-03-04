package org.supurdueper.robot2026.state;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.Setter;
import org.supurdueper.robot2026.RobotContainer;
import org.supurdueper.robot2026.subsystems.Shooter;
import org.supurdueper.robot2026.subsystems.ShooterHood;

public final class RobotStates {

    public static final Trigger sim = new Trigger(RobotBase::isSimulation);
    public static final Trigger teleop = RobotModeTriggers.teleop();
    public static final Trigger auto = RobotModeTriggers.autonomous();
    public static final Trigger disabled = RobotModeTriggers.disabled();
    public static final Driver driver = RobotContainer.getDriver();
    public static final Driver testController = RobotContainer.getTestController();
    public static final Shooter shooter = RobotContainer.getShooter();
    public static final ShooterHood hood = RobotContainer.getShooterHood();

    @Getter
    @Setter
    private static boolean autoAim = false;

    @Getter
    @Setter
    private static boolean autoIntake = false;

    @Getter
    @Setter
    private static boolean autoShoot = false;

    // auto
    public static final Trigger auto_intake = new Trigger(RobotStates::isAutoIntake).and(auto);
    public static final Trigger auto_aim = new Trigger(RobotStates::isAutoAim).and(auto);
    public static final Trigger auto_shoot = new Trigger(RobotStates::isAutoShoot).and(auto);

    // information
    public static final Trigger infoShooterAtSpeed = shooter.isAtVelocityTrigger();
    public static final Trigger infoHoodAtAngle = hood.isAtPositionTrigger();
    public static final Trigger infoReadyToShoot = infoHoodAtAngle.and(infoShooterAtSpeed);

    // Actions
    public static final Trigger rezeroFieldHeading = driver.select.and(teleop);
    public static final Trigger actionIntake = driver.leftBumper.and(teleop);
    public static final Trigger actionAim = driver.rightTrigger.and(teleop);
    public static final Trigger actionShoot = driver.rightBumper.and(teleop);

    public static final Trigger actionClimbPrep = driver.downDpad.and(teleop);
    public static final Trigger actionClimb = driver.leftDpad.and(teleop);

    public static final Trigger actionTestA = testController.A.and(teleop);
    public static final Trigger actionTestB = testController.B.and(teleop);
    public static final Trigger actionTestX = testController.X.and(teleop);
    public static final Trigger actionTestY = testController.Y.and(teleop);

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }
}
