// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.ArmAngles;
import frc.team4646.StabilityCounter;

public class ArmPosition extends CommandBase {
  private final ArmSubsystem subsystem = RobotContainer.ARMS;
  private final ArmAngles setpoint;
  private final StabilityCounter counterMain, counterSecondary;

  public ArmPosition(ArmAngles AnglesDesired){
    addRequirements(subsystem);
    this.setpoint = AnglesDesired;
    counterMain = new StabilityCounter(10);
    counterSecondary = new StabilityCounter(10);
  }

  public ArmPosition(double mainDegrees, double secondaryDegrees) {
    this(new ArmAngles(mainDegrees, secondaryDegrees));
  }

  @Override
  public void initialize() {
    counterMain.reset();
    counterSecondary.reset();
  }

  @Override
  public void execute() {
    counterMain.calculate(subsystem.isOnTargetPositionMain());
    counterSecondary.calculate(subsystem.isOnTargetPositionSecondary());
    //if(subsystem.isHomedPreviously()) {
      subsystem.setTargetPositionRaw(setpoint);  // calling repeatedly allows envelop to constantly compensate for current position
    //}
  }

  @Override
  public boolean isFinished() {
    return counterMain.isStable() && counterSecondary.isStable();
  }
}
