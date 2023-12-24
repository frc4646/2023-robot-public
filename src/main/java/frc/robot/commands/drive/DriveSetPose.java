// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class DriveSetPose extends InstantCommand {
  private final SwerveDriveSubsystem subsystem = RobotContainer.SWERVE;
  private final Pose2d pose;

  public DriveSetPose(Pose2d pose) {
    this.pose = pose;
  }

  public DriveSetPose(PathPlannerTrajectory trajectory) {
    this.pose = trajectory.getInitialHolonomicPose();
  }

  @Override
  public void initialize() {
    subsystem.resetOdometry(pose);
  }
}
