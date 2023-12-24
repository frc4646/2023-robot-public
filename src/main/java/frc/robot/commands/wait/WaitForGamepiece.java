// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wait;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Robotstate;
import frc.team4646.Canifier;
import frc.team4646.StabilityCounter;

public class WaitForGamepiece extends CommandBase {
  private final Intake intake = RobotContainer.INTAKE;
  private final Robotstate state = RobotContainer.STATE;
  private final Canifier canfiier = RobotContainer.CANIFIER;
  private final StabilityCounter counterCube, counterCone;

  public WaitForGamepiece() {
    counterCube = new StabilityCounter(10);
    counterCone = new StabilityCounter(2);
  }

  @Override
  public void initialize() {
    counterCube.reset();
    counterCone.reset();
  }

  @Override
  public void execute() {
    // boolean intake = subsystem.getCurrentAmps() > 30.0;
    // boolean intake = subsystem.getCurrentAmps() > 33.0;
    boolean hasCube = canfiier.isCubeDetected();
    boolean hasCone = canfiier.isConeDetected();
    counterCube.calculate(hasCube);
    counterCone.calculate(hasCone);

    RobotContainer.INFRASTRUCTURE.setArmsLEDs(!(state.isConeMode() && hasCone || state.isCubeMode() && hasCube));
  }

  @Override
  public boolean isFinished() {
    return state.isConeMode() ? counterCone.isStable() : counterCube.isStable();
  }
}
