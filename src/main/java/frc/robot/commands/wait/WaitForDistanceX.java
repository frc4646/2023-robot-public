// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wait;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class WaitForDistanceX extends CommandBase {
  private final SwerveDriveSubsystem swerveDrive = RobotContainer.SWERVE;
  private final double distanceX;
  
  public WaitForDistanceX(double distanceX) {
    this.distanceX = distanceX;
  }

  @Override
  public boolean isFinished() {
    return swerveDrive.getPose().getX() > distanceX;
  }
}
