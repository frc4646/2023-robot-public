// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.group;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class IfHasGamepiece extends ConditionalCommand {
  public IfHasGamepiece(Command command) {
    super(command, new WaitCommand(0.0), () -> { return IfHasGamepiece.hasGamepiece(); });
  }

  public static boolean hasGamepiece() {
    return RobotContainer.CANIFIER.isConeDetected() || RobotContainer.CANIFIER.isCubeDetected();
  }
}
