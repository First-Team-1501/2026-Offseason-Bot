package org.supurdueper.robot2026.state;

import static org.supurdueper.robot2026.Constants.DriverConstants.*;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverXbox extends DriverControls {

    private final CommandXboxController xbox = new CommandXboxController(kXboxDriverPort);

    @Override
    public Trigger rezeroHeading() {
        return xbox.back();
    }

    @Override
    public Trigger intake() {
        return xbox.leftTrigger(kXboxTriggerThreshold);
    }

    @Override
    public Trigger purge() {
        return xbox.leftBumper();
    }

    @Override
    public Trigger shoot() {
        return xbox.rightTrigger(kXboxTriggerThreshold);
    }

    @Override
    public Trigger aim() {
        return xbox.a();
    }

    @Override
    public Trigger setShot() {
        return xbox.x();
    }

    @Override
    public Trigger zeroHood() {
        return xbox.povDown();
    }

    @Override
    public Trigger defenseToggle() {
        return xbox.y();
    }

    @Override
    public Trigger hoodMax() {
        return xbox.rightBumper();
    }

    @Override
    protected double fwdAxis() {
        return -xbox.getLeftY();
    }

    @Override
    protected double leftAxis() {
        return -xbox.getLeftX();
    }

    @Override
    protected double ccwAxis() {
        return -xbox.getRightX();
    }
}
