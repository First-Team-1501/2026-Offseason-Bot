package org.supurdueper.robot2026.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static org.supurdueper.robot2026.Constants.DriveConstants.*;
import static org.supurdueper.robot2026.state.RobotStates.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.supurdueper.lib.LoggedTunableNumber;
// import org.supurdueper.lib.swerve.DriveToPose;
import org.supurdueper.lib.utils.AllianceFlip;
import org.supurdueper.robot2026.Constants;
import org.supurdueper.robot2026.RobotContainer;
import org.supurdueper.robot2026.state.DriverSticks;
import org.supurdueper.robot2026.subsystems.drive.DriveSysId.SysIdSwerveTranslationCurrent;
import org.supurdueper.robot2026.subsystems.drive.generated.TunerConstants;

public class DriveStates {

    private Drivetrain drivetrain;
    private DriverSticks driver;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second
    private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric();
    private final SysIdSwerveTranslationCurrent driveCurrentTuning = new SysIdSwerveTranslationCurrent();
    private final FieldCentricFacingAngle fieldCentricFacingAngle = new FieldCentricFacingAngle();
    private final AutoAim driveFacingHub = new AutoAim();
    // Heading gains, adjustable live in tuning mode
    private final LoggedTunableNumber headingKpTunable = new LoggedTunableNumber("AutoAim/Kp", headingKp);
    private final LoggedTunableNumber headingKiTunable = new LoggedTunableNumber("AutoAim/Ki", headingKi);
    private final LoggedTunableNumber headingKdTunable = new LoggedTunableNumber("AutoAim/Kd", headingKd);
    // Heading tuning telemetry (DataLog): aim active at 50 Hz, controller configuration at 1 Hz
    private final BooleanLogEntry headingActive =
            new BooleanLogEntry(DataLogManager.getLog(), "/Tuning/Heading/Active");
    private final StringLogEntry headingConfig =
            new StringLogEntry(DataLogManager.getLog(), "/Tuning/Heading/Config", "", "json");
    private int periodicCount = 0;

    public DriveStates(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.driver = RobotContainer.getDriver();
        fieldCentricFacingAngle.HeadingController.setPID(headingKp, headingKi, headingKd);
        fieldCentricFacingAngle.RotationalDeadband = rotationClosedLoopDeadband.in(RadiansPerSecond);
        fieldCentricFacingAngle.MaxAbsRotationalRate = MaxAngularRate;
        driveFacingHub.HeadingController.setPID(headingKp, headingKi, headingKd);
        driveFacingHub.RotationalDeadband = rotationClosedLoopDeadband.in(RadiansPerSecond);
        driveFacingHub.MaxAbsRotationalRate = MaxAngularRate;
    }

    /** Heading telemetry; in tuning mode also applies dashboard heading gains (called from Drivetrain.periodic). */
    public void periodic() {
        headingActive.update(actionAim.getAsBoolean() || auto_aim.getAsBoolean());
        if (++periodicCount % 50 == 0) {
            headingConfig.update("{\"kP\":" + driveFacingHub.HeadingController.getP()
                    + ",\"kI\":" + driveFacingHub.HeadingController.getI()
                    + ",\"kD\":" + driveFacingHub.HeadingController.getD()
                    + ",\"maxRateRadPerSec\":" + driveFacingHub.MaxAbsRotationalRate
                    + ",\"lockEnterDeg\":" + kAimLockEnterDeg
                    + ",\"lockExitDeg\":" + kAimLockExitDeg
                    + ",\"lockMaxTranslationMps\":" + kAimLockMaxTranslationMps + "}");
        }
        if (!Constants.tuningMode) {
            return;
        }
        LoggedTunableNumber.ifChanged(
                hashCode(),
                gains -> {
                    fieldCentricFacingAngle.HeadingController.setPID(gains[0], gains[1], gains[2]);
                    driveFacingHub.HeadingController.setPID(gains[0], gains[1], gains[2]);
                    DogLog.log("AutoAim/Applied Gains", gains);
                },
                headingKpTunable,
                headingKiTunable,
                headingKdTunable);
    }

    public void bindCommands() {
        drivetrain.setDefaultCommand(normalTeleopDrive());
        rezeroFieldHeading.onTrue(
                Commands.runOnce(() -> drivetrain.resetRotation(AllianceFlip.apply(Rotation2d.kZero))));
        actionAim.whileTrue(driveFacingHub());
        auto_aim.whileTrue(driveFacingHub());
    }

    private Command normalTeleopDrive() {
        return drivetrain.applyRequest(() -> driveFieldCentric
                .withVelocityX(driver.getDriveFwdPositive() * MaxSpeed)
                .withVelocityY(driver.getDriveLeftPositive() * MaxSpeed)
                .withRotationalRate(driver.getDriveCCWPositive() * MaxAngularRate)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
    }

    public Command setVelocityDrive(double velocity) {
        return drivetrain.applyRequest(
                () -> driveFieldCentric.withVelocityX(velocity).withDriveRequestType(DriveRequestType.Velocity));
    }

    private Command currentTuningDrive() {
        return drivetrain.applyRequest(() -> driveCurrentTuning.withCurrent(driver.getDriveFwdPositive() * 40));
    }

    private Command driveFacingAngle(Rotation2d angle) {
        return drivetrain.applyRequest(() -> fieldCentricFacingAngle
                .withVelocityX(driver.getDriveFwdPositive() * MaxSpeed)
                .withVelocityY(driver.getDriveLeftPositive() * MaxSpeed)
                .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                .withTargetDirection(angle)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
    }

    private Command driveFacingHub() {
        // X-lock once aimed while shooting (AutoAim)
        return drivetrain.applyRequest(() -> driveFacingHub
                .withLockWhenAimed(actionShoot.getAsBoolean() || auto_shoot.getAsBoolean())
                .withVelocityX(driver.getDriveFwdPositive() * MaxSpeed)
                .withVelocityY(driver.getDriveLeftPositive() * MaxSpeed)
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
    }
}
