// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Robotstate;
import frc.team4646.Canifier;

public class IntakePulseIn extends CommandBase {
  private final Intake intake = RobotContainer.INTAKE;
  private final Canifier canifier = RobotContainer.CANIFIER;
  private final Robotstate state = RobotContainer.STATE;

  public IntakePulseIn() {
    addRequirements(intake);
  }

  @Override
  public void execute() {
    double setpoint = 0;
    if(state.isConeMode() && canifier.isConeDetected()) {
      boolean isStalled = intake.getCurrentAmps() > 45.0;
      setpoint = isStalled ? 0.0 : Constants.INTAKE.CONE_IN_PERCENT;
    }
    intake.setMotor(setpoint);
  }
}
