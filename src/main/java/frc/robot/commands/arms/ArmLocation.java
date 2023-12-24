// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.IntakeLocation;

public class ArmLocation extends CommandBase {
  private final ArmSubsystem subsystem = RobotContainer.ARMS;
  private final IntakeLocation setpoint;

  public ArmLocation(IntakeLocation intakeLocation) {
    addRequirements(subsystem);
    this.setpoint = intakeLocation;
  }

  @Override
  public void execute() {
    subsystem.setTargetLocation(setpoint);  // calling repeatedly allows envelop to constantly compensate for current position
  }

  @Override
  public boolean isFinished() {
    return subsystem.isOnTargetPositionMain() && subsystem.isOnTargetPositionSecondary();
  }
}
