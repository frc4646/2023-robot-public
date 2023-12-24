// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.stinger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Stinger;

public class CubeFlip extends CommandBase {
  private final Stinger subsystem = RobotContainer.STINGER;
  private boolean flip;
  
  public CubeFlip(boolean flip) {
    addRequirements(subsystem);
    this.flip = flip;
  }

  @Override
  public void initialize() {
    if(flip) {
      subsystem.flip();
    } else {
      subsystem.reset();
    }
  }

  @Override
  public boolean isFinished() {
    return true ;
  }
}
