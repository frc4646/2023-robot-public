// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arms;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class IfArmSecondaryBelow extends ConditionalCommand {
  public IfArmSecondaryBelow(Command command, double angle) {
    super(command, new WaitCommand(0.0), () -> { return IfArmSecondaryBelow.isBelow(angle); });
  }

  public static boolean isBelow(double angle) {
    return RobotContainer.ARMS.getAngleSecondaryRaw().getDegrees() < angle;
  }
}
