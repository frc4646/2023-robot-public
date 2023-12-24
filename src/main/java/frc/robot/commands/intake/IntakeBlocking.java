// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeBlocking extends CommandBase {
  Intake subsystem = RobotContainer.INTAKE;
  double percentOutput;

  /** Creates a new teleopIntake. */
  public IntakeBlocking(double percentOutput) {
    this.percentOutput = percentOutput;
    addRequirements(subsystem);
  }

  public IntakeBlocking() {
    this(0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      subsystem.setMotor(percentOutput);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
