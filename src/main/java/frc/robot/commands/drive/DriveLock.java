// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveLock extends SequentialCommandGroup {
  public DriveLock() {
    addCommands(
      new DriveLockTurning().withTimeout(0.25),
      new DriveOpenLoop(new Translation2d())
    );
  }
}
