// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveOpenLoop;
import frc.robot.commands.wait.WaitForPitch;
import frc.robot.commands.wait.WaitForPitch.DIRECTION;

public class DrivePitch extends ParallelRaceGroup {
  public DrivePitch(double speed, DIRECTION direction, double degrees, int counts, double timeout) {
    addCommands(
      new DriveOpenLoop(new Translation2d(speed, 0.0)),
      new WaitForPitch(direction, degrees, counts),
      new WaitCommand(timeout)
    );
  }
}
