// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.WristSubsystem;
import frc.team4646.StabilityCounter;

public class WristPosition extends CommandBase {
  private final WristSubsystem subsystem = RobotContainer.WRIST;
  private final double setpoint;
  private final StabilityCounter counter;

  public WristPosition(double setpoint) {
    addRequirements(subsystem);
    this.setpoint = setpoint;
    counter = new StabilityCounter(10);
  }

  @Override
  public void initialize() {
    counter.reset();
    subsystem.setPosition(setpoint);
  }

  @Override
  public void execute() {
    counter.calculate(subsystem.isOnTarget());
  }

  @Override
  public boolean isFinished() {
    return counter.isStable();
  }
}
