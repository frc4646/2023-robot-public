// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class DriveTurnToAngle extends CommandBase {
  private ProfiledPIDController snapPIDController = new ProfiledPIDController(3.0, 5.0, 0.0, new Constraints(Constants.SWERVE_DRIVE.MAX_ROTATION_SPEED_DPS, Constants.SWERVE_DRIVE.MAX_ROTATION_ACCEL_DPSS));
  private SimpleMotorFeedforward snapFeedForward = new SimpleMotorFeedforward(0.0, 0.0);

  private final SwerveDriveSubsystem subsystem = RobotContainer.SWERVE;
  private final double fieldRelativeDegrees, toleranceDegrees;

  public DriveTurnToAngle(double fieldRelativeDegrees, double toleranceDegrees) {
    addRequirements(subsystem);
    this.fieldRelativeDegrees = fieldRelativeDegrees;
    this.toleranceDegrees = toleranceDegrees;    
    snapPIDController.setGoal(new TrapezoidProfile.State(fieldRelativeDegrees, 0.0));
  }

  @Override
  public void initialize() {
    snapPIDController.reset(subsystem.getGyroAngle().getDegrees());
  }

  @Override 
  public void execute() {
    Translation2d move = new Translation2d(-0.4, 0.0);
    double setpoint = snapPIDController.calculate(subsystem.getGyroAngle().getDegrees());
    double feedForward = snapFeedForward.calculate(snapPIDController.getSetpoint().velocity);

    subsystem.drive(move, setpoint + feedForward, true, true);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(subsystem.getGyroAngle().getDegrees() - fieldRelativeDegrees) < toleranceDegrees;
  }
}
