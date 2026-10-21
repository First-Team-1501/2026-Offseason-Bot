package org.supurdueper.robot2026.state;

import static org.supurdueper.robot2026.Constants.DriverConstants.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverSticks extends DriverControls {

    private final Joystick driveStick = new Joystick(kDriveStickPort);
    private final Joystick rotationStick = new Joystick(kRotationStickPort);

    @Override
    public Trigger rezeroHeading() {
        return new JoystickButton(driveStick, 1);
    }

    @Override
    public Trigger intake() {
        return new JoystickButton(driveStick, 16);
    }

    @Override
    public Trigger purge() {
        return new JoystickButton(driveStick, 17);
    }

    @Override
    public Trigger shoot() {
        return new JoystickButton(driveStick, 18);
    }

    @Override
    public Trigger aim() {
        return new JoystickButton(rotationStick, 18);
    }

    @Override
    public Trigger setShot() {
        return new JoystickButton(rotationStick, 17);
    }

    @Override
    public Trigger zeroHood() {
        return new JoystickButton(rotationStick, 1);
    }

    @Override
    protected double fwdAxis() {
        return -driveStick.getY();
    }

    @Override
    protected double leftAxis() {
        return -driveStick.getX();
    }

    @Override
    protected double ccwAxis() {
        return -rotationStick.getX();
    }
}
