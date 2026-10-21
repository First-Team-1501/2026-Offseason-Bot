package org.supurdueper.robot2026.state;

import static org.supurdueper.robot2026.Constants.DriverConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.supurdueper.lib.LoggedTunableNumber;

/**
 * Driver input. DriverConstants.kXboxDriver selects DriverXbox (port 0) or DriverSticks (ports 0 and 1). Subclasses map
 * the buttons and raw axes; the stick response is applied here for both.
 */
public abstract class DriverControls {

    // Stick response, live in tuning mode
    private static final LoggedTunableNumber stickDeadband =
            new LoggedTunableNumber("Driver/Stick Deadband", kStickDeadband);
    private static final LoggedTunableNumber stickExponent =
            new LoggedTunableNumber("Driver/Stick Exponent", kStickExponent);

    public abstract Trigger rezeroHeading();

    public abstract Trigger intake();

    public abstract Trigger purge();

    public abstract Trigger shoot();

    public abstract Trigger aim();

    public abstract Trigger setShot();

    public abstract Trigger zeroHood();

    /** Raw translation axis, -1 to 1, forward positive. */
    protected abstract double fwdAxis();

    /** Raw translation axis, -1 to 1, left positive. */
    protected abstract double leftAxis();

    /** Raw rotation axis, -1 to 1, counter-clockwise positive. */
    protected abstract double ccwAxis();

    public double getDriveFwdPositive() {
        return shape(fwdAxis()) * kSlowModeScalor;
    }

    public double getDriveLeftPositive() {
        return shape(leftAxis()) * kSlowModeScalor;
    }

    public double getDriveCCWPositive() {
        return shape(ccwAxis()) * kSlowModeScalor;
    }

    /** Power curve from the 1501 2026 comp bot (DriveSticks.applyExponential): deadband, rescale, exponent. */
    private static double shape(double input) {
        double deadband = MathUtil.clamp(stickDeadband.get(), 0.0, 0.3);
        double exponent = MathUtil.clamp(stickExponent.get(), 1.0, 5.0);
        double magnitude = Math.abs(input);
        if (magnitude < deadband) {
            return 0.0;
        }
        return Math.signum(input) * Math.pow((magnitude - deadband) / (1.0 - deadband), exponent);
    }
}
