// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class DriveLockTurning extends CommandBase {
  private final SwerveDriveSubsystem subsystem = RobotContainer.SWERVE;
  private final SwerveModuleState[] states;
  
  public DriveLockTurning() {
      addRequirements(subsystem);
      final double percentRotationPivotWheelsButNotRobot = 0.02;
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, Math.toRadians(percentRotationPivotWheelsButNotRobot * Constants.SWERVE_DRIVE.MAX_ROTATION_SPEED_DPS));
      states = Constants.SWERVE_DRIVE.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
  }

  @Override
  public void initialize() {
    subsystem.setDesiredStates(states);
  }
}