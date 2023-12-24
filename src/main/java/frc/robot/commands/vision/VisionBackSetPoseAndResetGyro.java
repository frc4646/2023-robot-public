// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.AprilTagData;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class VisionBackSetPoseAndResetGyro extends InstantCommand {
  private final Vision subsystem = RobotContainer.BACK_VISION;
  private final SwerveDriveSubsystem swerve = RobotContainer.SWERVE;

  @Override
  public void initialize() {
    AprilTagData data = subsystem.getRobotPositionFromApriltag();
    swerve.resetGyroAndOdometry(data.pose);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
