package org.supurdueper.robot2026.state;

import static org.supurdueper.robot2026.Constants.DriverConstants.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverSticks {

    private final Joystick driveStick = new Joystick(kDriveStickPort);
    private final Joystick rotationStick = new Joystick(kRotationStickPort);

    public final Trigger rezeroHeading = new JoystickButton(driveStick, 1);
    public final Trigger setShot = new JoystickButton(driveStick, 16);
    public final Trigger purge = new JoystickButton(driveStick, 17);
    public final Trigger aim = new JoystickButton(driveStick, 18);
    public final Trigger zeroHood = new JoystickButton(rotationStick, 1);
    public final Trigger intake = new JoystickButton(rotationStick, 17);
    public final Trigger shoot = new JoystickButton(rotationStick, 18);

    public double getDriveFwdPositive() {
        return shape(-driveStick.getY()) * kSlowModeScalor;
    }

    public double getDriveLeftPositive() {
        return shape(-driveStick.getX()) * kSlowModeScalor;
    }

    public double getDriveCCWPositive() {
        return shape(-rotationStick.getX()) * kSlowModeScalor;
    }

    private static double shape(double input) {
        double magnitude = Math.abs(input);
        if (magnitude < kStickDeadband) {
            return 0.0;
        }
        return Math.signum(input) * Math.pow((magnitude - kStickDeadband) / (1.0 - kStickDeadband), kStickExponent);
    }
}
