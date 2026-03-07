package org.supurdueper.robot2026.state;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.Setter;
import org.supurdueper.robot2026.RobotContainer;
import org.supurdueper.robot2026.subsystems.Shooter;
import org.supurdueper.robot2026.subsystems.ShooterHood;

import dev.doglog.DogLog;

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

    @Getter
    @Setter
    private static boolean autoDropIntake = false;

    // auto
    public static final Trigger auto_intake = new Trigger(RobotStates::isAutoIntake).and(auto);
    public static final Trigger auto_aim = new Trigger(RobotStates::isAutoAim).and(auto);
    public static final Trigger auto_shoot = new Trigger(RobotStates::isAutoShoot).and(auto);
    public static final Trigger auto_drop_intake = new Trigger(RobotStates::isAutoDropIntake).and(auto);

    // information
    public static final Trigger infoShooterAtSpeed = shooter.isAtVelocityTrigger();
    public static final Trigger infoHoodAtAngle = hood.isAtPositionTrigger();
    public static final Trigger infoReadyToShoot = infoHoodAtAngle.and(infoShooterAtSpeed);

    // Actions
    public static final Trigger rezeroFieldHeading = driver.select.and(teleop);
    public static final Trigger actionIntake = driver.leftBumper.and(teleop);
    public static final Trigger actionAim = driver.rightTrigger.and(teleop);
    public static final Trigger actionShoot = driver.rightBumper.and(teleop);
    public static final Trigger actionPurge = driver.leftTrigger.and(teleop);

    public static final Trigger actionClimb = driver.B.and(teleop);
    public static final Trigger actionClimberUp = driver.Y.and(teleop);
    public static final Trigger actionClimberHome = driver.A.and(teleop);

    public static final Trigger actionTestA = testController.A.and(teleop);
    public static final Trigger actionTestB = testController.B.and(teleop);
    public static final Trigger actionTestX = testController.X.and(teleop);
    public static final Trigger actionTestY = testController.Y.and(teleop);

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }

    public static void log() {
        DogLog.log("States/AutoIntake", auto_intake.getAsBoolean());
        DogLog.log("States/AutoAim", auto_aim.getAsBoolean());
        DogLog.log("States/AutoShoot", auto_shoot.getAsBoolean());
        DogLog.log("States/AutoDropIntake", auto_drop_intake.getAsBoolean());
        DogLog.log("States/InfoShooterAtSpeed", infoShooterAtSpeed.getAsBoolean());
        DogLog.log("States/InfoHoodAtAngle", infoHoodAtAngle.getAsBoolean());
        DogLog.log("States/InfoReadyToShoot", infoReadyToShoot.getAsBoolean());
        DogLog.log("States/ActionAim", actionAim.getAsBoolean());
        DogLog.log("States/ActionClimb", actionClimb.getAsBoolean());
        DogLog.log("States/ActionClimbUp", actionClimb.getAsBoolean());
        DogLog.log("States/ActionClimbHome", actionClimberHome.getAsBoolean());
        DogLog.log("States/ActionShoot", actionShoot.getAsBoolean());
        DogLog.log("States/ActionPurge", actionPurge.getAsBoolean());
        DogLog.log("States/ActionIntake", actionIntake.getAsBoolean());
        DogLog.log("States/ActionRezeroFieldHeading", rezeroFieldHeading.getAsBoolean());
    }
}
