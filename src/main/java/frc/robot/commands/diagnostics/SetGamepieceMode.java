// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.diagnostics;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Robotstate;
import frc.robot.util.GAMEPIECE_MODE;

public class SetGamepieceMode extends InstantCommand {
  private final Robotstate subsystem = RobotContainer.STATE;
  private final GAMEPIECE_MODE mode;
  public SetGamepieceMode(GAMEPIECE_MODE mode) {
    this.mode = mode;
  }

  @Override
  public void initialize() {
    if(mode != GAMEPIECE_MODE.NONE) {
      subsystem.setMode(mode);
    }
  }
}
