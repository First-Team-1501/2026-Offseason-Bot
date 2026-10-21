package org.supurdueper.robot2026.state;

import static org.supurdueper.robot2026.Constants.DriverConstants.*;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.Setter;
import org.supurdueper.lib.gamepad.Gamepad;

// 7457's driver Xbox class, kept as the test controller (port 3); the driver input is DriverControls
public class TestController extends Gamepad {

    // // Triggers, these would be robot states such as ampReady, intake, visionAim,
    // etc.
    // // If triggers need any of the config values set them in the constructor
    // /* A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simulation */

    public final Trigger extraRight = rightStickClick;
    public final Trigger extraLeft = leftStickClick;

    // /* Use the right stick to set a cardinal direction to aim at */
    public final Trigger driving;
    public final Trigger steer;

    // // DISABLED TRIGGERS
    // public final Trigger coastOn_dB = disabled.and(B);
    // public final Trigger coastOff_dA = disabled.and(A);

    @Getter
    @Setter
    private boolean isSlowMode = true;

    @Getter
    @Setter
    private boolean isTurboMode = false;

    /** Create a new test controller on the given port. */
    public TestController(int port) {
        super(port, kLeftStickCurve, kDeadzone, kRightStickCurve, kDeadzone, kTriggerCurve, kDeadzone);
        driving = RobotModeTriggers.teleop().and(leftStickX.or(leftStickY));
        steer = RobotModeTriggers.teleop().and(rightStickX.or(rightStickY));
    }

    // DRIVE METHODS
    public void setMaxVelocity(double maxVelocity) {
        leftStickCurve.setScalar(maxVelocity);
    }

    public void setMaxRotationalVelocity(double maxRotationalVelocity) {
        rightStickCurve.setScalar(maxRotationalVelocity);
    }

    // Positive is forward, up on the left stick is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveFwdPositive() {
        double fwdPositive = leftStickCurve.calculate(-1 * getLeftY());
        if (isSlowMode) {
            fwdPositive *= Math.abs(kSlowModeScalor);
        }
        return fwdPositive;
    }

    // Positive is left, left on the left stick is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveLeftPositive() {
        double leftPositive = -1 * leftStickCurve.calculate(getLeftX());
        if (isSlowMode) {
            leftPositive *= Math.abs(kSlowModeScalor);
        }
        return leftPositive;
    }

    // Positive is counter-clockwise, left Trigger is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveCCWPositive() {
        double ccwPositive = rightStickCurve.calculate(getRightX());
        if (isSlowMode) {
            ccwPositive *= Math.abs(kSlowModeScalor);
        } else if (isTurboMode) {
            ccwPositive *= Math.abs(kTurboModeScalor);
        } else {
            ccwPositive *= Math.abs(kDefaultTurnScalor);
        }
        return -1 * ccwPositive; // invert the value
    }
}
