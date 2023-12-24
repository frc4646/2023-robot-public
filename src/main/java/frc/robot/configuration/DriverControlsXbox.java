package frc.robot.configuration;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.team4646.Util;

public class DriverControlsXbox implements IDriverControls {
    private final double kJoystickDeadband = .2;
    private final int kJoystickPower = 2; // square or cube the input to get finer control at low input values
    private final CommandXboxController controller = new CommandXboxController(0);

    public DriverControlsXbox() {
        // Reset Gyro
        controller.y().onTrue(new InstantCommand(RobotContainer.SWERVE::resetGyro, RobotContainer.SWERVE));

        // TODO use ABXY for snapping to angle, so you can still use your left hand to translate
        // controller.y().whileTrue(new SnapToAngleDrive(controller, 0));
    }

    @Override
    public double getX() {
        return -Util.handleJoystick(controller.getLeftY(), kJoystickDeadband, kJoystickPower); 
    }

    @Override
    public double getY() {
        return -Util.handleJoystick(controller.getLeftX(), kJoystickDeadband, kJoystickPower); 
    }

    @Override
    public double getRotation() {
        return -Util.handleJoystick(controller.getRightX(), kJoystickDeadband, kJoystickPower); 
    }

    @Override
    public boolean getFieldRelativeToggle() {
        return controller.getHID().getAButtonPressed();
    }

    @Override
    public boolean getSpeedBoost() {
        return false;
    }

    @Override
    public boolean getSpeedSlow() {
        return false;
    }

    @Override
    public int getSnapAngle() {
        return -1;
    }
}
