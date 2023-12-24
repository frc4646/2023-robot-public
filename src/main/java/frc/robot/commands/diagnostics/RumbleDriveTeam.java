// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.diagnostics;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.RobotContainer;
import frc.robot.configuration.OperatorControls;

public class RumbleDriveTeam extends ScheduleCommand {
  private final OperatorControls controls = RobotContainer.operator;

  public RumbleDriveTeam(double timeout) {
    this.withTimeout(timeout);
  }

  @Override
  public void initialize() {
    controls.setRumble(true, 0.5);
  }

  @Override
  public void end(boolean interrupted) {
    controls.setRumble(false, 0.0);
  }
}
