// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.group;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.group.cone.ConeOut;
import frc.robot.commands.group.cube.CubeOut;

public class GamepieceOut extends ConditionalCommand {
  public GamepieceOut() {
    super(new ConeOut(), new CubeOut(), RobotContainer.STATE::isConeMode);
  }
}
