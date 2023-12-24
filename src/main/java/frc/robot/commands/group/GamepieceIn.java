// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.group;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.group.cone.ConeIn;
import frc.robot.commands.group.cube.CubeIn;

public class GamepieceIn extends ConditionalCommand {
  public GamepieceIn() {
    super(new ConeIn(), new CubeIn(), RobotContainer.STATE::isConeMode);
  }
}
