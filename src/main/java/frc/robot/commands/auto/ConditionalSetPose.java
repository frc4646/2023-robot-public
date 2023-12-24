// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveSetPose;

public class ConditionalSetPose extends ConditionalCommand {
  public ConditionalSetPose(PathPlannerTrajectory trajectoryBlue, PathPlannerTrajectory trajectoryRed) {
    super(
      new DriveSetPose(trajectoryBlue),
      new DriveSetPose(trajectoryRed),
      RobotContainer.STATE::isBlueAlliance
    );
  }
}
