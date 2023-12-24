// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.group.cone;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arms.ArmPosition;
import frc.robot.commands.intake.IntakeInstant;
import frc.robot.commands.wait.WaitForGamepiece;
import frc.robot.commands.wrist.WristPosition;

public class ConeDoubleSubstation extends SequentialCommandGroup {
  public ConeDoubleSubstation() {
    addCommands(
      new WaitForGamepiece().deadlineWith(
        new ConeIn(),
        new ArmPosition(Constants.STATE.WALL_CONE.arms).deadlineWith(
          new WristPosition(Constants.STATE.WALL_CONE.wrist)
        )
      ),
      // new ArmPosition(Constants.STATE.WALL_CONE.arms.mainDegrees, -10.0),
      new ConeIn(.5),
      new WristPosition(Constants.STATE.TRANSPORT.wrist).withTimeout(2.0),
      new ArmPosition(Constants.STATE.TRANSPORT.arms),
      new IntakeInstant()
    );
  }
}
