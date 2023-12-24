// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveTrajectory;

public class ConditionalDriveTrajectory extends ConditionalCommand {
  public ConditionalDriveTrajectory(PathPlannerTrajectory trajectoryBlue, PathPlannerTrajectory trajectoryRed) {
    super(
      new DriveTrajectory(trajectoryBlue),
      new DriveTrajectory(trajectoryRed),
      RobotContainer.STATE::isBlueAlliance
    );
  }
}
