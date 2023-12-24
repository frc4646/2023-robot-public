// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.group.cone;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.intake.IntakeInstant;

public class ConeEject extends SequentialCommandGroup {
  public ConeEject() {
    addCommands(
      new IntakeInstant(-Constants.INTAKE.CONE_IN_PERCENT),
      new WaitCommand(2.0),
      new IntakeInstant()
    );
  }
}
