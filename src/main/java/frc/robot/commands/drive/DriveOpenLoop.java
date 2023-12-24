// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class DriveOpenLoop extends CommandBase {
  private final SwerveDriveSubsystem subsystem = RobotContainer.SWERVE;
  private final Translation2d move;

  public DriveOpenLoop(Translation2d move) {
    addRequirements(subsystem);
    this.move = move;
  }

  public DriveOpenLoop() {
    this(new Translation2d(0, 0));
  }

  @Override
  public void initialize() {
    subsystem.setBrakeMode(true);
    subsystem.drive(move, 0.0, false, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
