package org.supurdueper.robot2026.state;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.Setter;
import org.supurdueper.lib.gamepad.Gamepad;
import org.supurdueper.robot2026.RobotContainer;
import org.supurdueper.robot2026.subsystems.Shooter;
import org.supurdueper.robot2026.subsystems.ShooterHood;
import org.supurdueper.robot2026.utils.ShotCalculator;

public final class RobotStates {

    public static final Trigger sim = new Trigger(RobotBase::isSimulation);
    public static final Trigger teleop = RobotModeTriggers.teleop();
    public static final Trigger auto = RobotModeTriggers.autonomous();
    public static final Trigger disabled = RobotModeTriggers.disabled();
    public static final DriverControls driver = RobotContainer.getDriver();
    public static final TestController testController = RobotContainer.getTestController();
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

    @Getter
    @Setter
    private static boolean autoRev = false;

    @Getter
    @Setter
    private static boolean autoDropHood = false;

    /**
     * The auto flags are static and the routines leave some set (autoDropIntake/autoDropHood always, aim/shoot when
     * auto ends mid-volley); stale flags fire their auto_* bindings at the start of the next auto in the same code run.
     * Called at the start of auto and teleop.
     */
    public static void clearAutoFlags() {
        autoAim = false;
        autoIntake = false;
        autoShoot = false;
        autoDropIntake = false;
        autoRev = false;
        autoDropHood = false;
    }

    // Defense mode (driver toggle, teleop; cleared when auto starts): intake, hopper, feeder, uptake and shooter
    // neutral and their actions off; drive and hood (stow, trench, max-angle hold) keep working
    @Getter
    private static boolean defenseMode = false;

    private static final String kDefenseKey = "Defense Mode";

    static {
        SmartDashboard.putBoolean(kDefenseKey, defenseMode);
    }

    // auto
    public static final Trigger auto_intake = new Trigger(RobotStates::isAutoIntake).and(auto);
    public static final Trigger auto_aim = new Trigger(RobotStates::isAutoAim).and(auto);
    public static final Trigger auto_shoot = new Trigger(RobotStates::isAutoShoot).and(auto);
    public static final Trigger auto_drop_intake = new Trigger(RobotStates::isAutoDropIntake).and(auto);
    public static final Trigger auto_rev = new Trigger(RobotStates::isAutoRev).and(auto);
    public static final Trigger auto_drop_hood = new Trigger(RobotStates::isAutoDropHood).and(auto);

    // information
    public static final Trigger infoShooterAtSpeed = shooter.isAtVelocityTrigger();
    public static final Trigger infoHoodAtAngle = hood.isAtPositionTrigger();
    public static final Trigger infoReadyToShoot = infoHoodAtAngle.and(infoShooterAtSpeed);
    public static final Trigger infoInRange = new Trigger(
            () -> ShotCalculator.inRange(RobotContainer.getDrivetrain().getState().Pose));

    // Actions
    public static final Trigger defense = new Trigger(RobotStates::isDefenseMode);
    private static final Trigger mechanisms = teleop.and(defense.negate());
    public static final Trigger rezeroFieldHeading = driver.rezeroHeading().and(teleop);
    public static final Trigger actionIntake = driver.intake().and(mechanisms);
    public static final Trigger actionAim = driver.aim().and(mechanisms);
    public static final Trigger actionShoot = driver.shoot().and(mechanisms);
    public static final Trigger actionPurge = driver.purge().and(mechanisms);
    public static final Trigger actionSetShot = driver.setShot().and(mechanisms);
    public static final Trigger actionZeroHood = driver.zeroHood();
    public static final Trigger actionDefenseToggle = driver.defenseToggle().and(teleop);
    // Hood to its max angle while held, also in defense mode (the trench zone keeps it down)
    public static final Trigger actionHoodMax = driver.hoodMax().and(teleop);

    public static final Trigger actionClimb = Gamepad.kFalse;
    public static final Trigger actionClimberUp = Gamepad.kFalse;
    public static final Trigger actionClimberHome = Gamepad.kFalse;

    public static final Trigger actionTestA = testController.A.and(teleop);
    public static final Trigger actionTestB = testController.B.and(teleop);
    public static final Trigger actionTestX = testController.X.and(teleop);
    public static final Trigger actionTestY = testController.Y.and(teleop);

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }

    /** Also shown on the dashboard as "Defense Mode". */
    public static void setDefenseMode(boolean on) {
        defenseMode = on;
        SmartDashboard.putBoolean(kDefenseKey, on);
    }

    public static void log() {
        DogLog.log("States/Defense", defenseMode);
        DogLog.log("States/ActionHoodMax", actionHoodMax.getAsBoolean());
        DogLog.log("States/AutoIntake", auto_intake.getAsBoolean());
        DogLog.log("States/AutoRev", auto_rev.getAsBoolean());
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
        DogLog.log("States/ActionSetShot", actionSetShot.getAsBoolean());
        DogLog.log("States/ActionRezeroFieldHeading", rezeroFieldHeading.getAsBoolean());
    }
}
