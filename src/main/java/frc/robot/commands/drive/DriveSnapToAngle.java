// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class DriveSnapToAngle extends CommandBase {
    SwerveDriveSubsystem swerve = RobotContainer.SWERVE;
    private ProfiledPIDController snapPIDController = new ProfiledPIDController(3.0, 5.0, 0.0,
            new Constraints(Constants.SWERVE_DRIVE.MAX_ROTATION_SPEED_DPS, Constants.SWERVE_DRIVE.MAX_ROTATION_ACCEL_DPSS));

    private SimpleMotorFeedforward snapFeedForward = new SimpleMotorFeedforward(0.0, 0.0);
    private boolean fieldRelative = true;

    public DriveSnapToAngle() {
        addRequirements(swerve);
        snapPIDController.enableContinuousInput(0, 360);
    }

    @Override
    public void initialize() {
        snapPIDController.reset(swerve.getGyroAngle().getDegrees());
    }

    @Override
    public void execute() {
        int angle = RobotContainer.driver.getSnapAngle();
        if(angle == -1) {
          angle = (int)swerve.getGyroAngle().getDegrees();
        }
        snapPIDController.setGoal(new TrapezoidProfile.State(angle, 0.0));

        if (RobotContainer.driver.getFieldRelativeToggle()) {
            fieldRelative = !fieldRelative;
        }

        double setpoint = snapPIDController.calculate(swerve.getGyroAngle().getDegrees());
        double feedForward = snapFeedForward.calculate(snapPIDController.getSetpoint().velocity);

        swerve.drive(DriveTeleopHelper.getMove(), setpoint + feedForward, fieldRelative, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
