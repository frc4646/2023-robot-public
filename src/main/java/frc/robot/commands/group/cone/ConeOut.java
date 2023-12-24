// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.group.cone;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.intake.IntakeBlocking;

public class ConeOut extends SequentialCommandGroup {
  public ConeOut() {
    addCommands(
      new IntakeBlocking(Constants.INTAKE.CONE_OUT_PERCENT)
    );
  }
}
