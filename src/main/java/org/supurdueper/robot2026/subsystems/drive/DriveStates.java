package org.supurdueper.robot2026.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static org.supurdueper.robot2026.Constants.DriveConstants.*;
import static org.supurdueper.robot2026.state.RobotStates.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import org.supurdueper.lib.swerve.DriveToPose;
import org.supurdueper.lib.utils.AllianceFlip;
import org.supurdueper.robot2026.RobotContainer;
import org.supurdueper.robot2026.state.Driver;
import org.supurdueper.robot2026.subsystems.drive.DriveSysId.SysIdSwerveTranslationCurrent;
import org.supurdueper.robot2026.subsystems.drive.generated.TunerConstants;

public class DriveStates {

    private Drivetrain drivetrain;
    private Driver driver;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second
    private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric();
    private final SysIdSwerveTranslationCurrent driveCurrentTuning = new SysIdSwerveTranslationCurrent();
    private final FieldCentricFacingAngle fieldCentricFacingAngle = new FieldCentricFacingAngle();
    private final AutoAim driveFacingHub = new AutoAim();

    public DriveStates(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.driver = RobotContainer.getDriver();
        fieldCentricFacingAngle.HeadingController.setPID(headingKp, headingKi, headingKd);
        fieldCentricFacingAngle.RotationalDeadband = rotationClosedLoopDeadband.in(RadiansPerSecond);
        driveFacingHub.HeadingController.setPID(headingKp, headingKi, headingKd);
        driveFacingHub.RotationalDeadband = rotationClosedLoopDeadband.in(RadiansPerSecond);
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
        return drivetrain.applyRequest(() -> driveFacingHub
                .withVelocityX(driver.getDriveFwdPositive() * MaxSpeed)
                .withVelocityY(driver.getDriveLeftPositive() * MaxSpeed)
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
    }
}
