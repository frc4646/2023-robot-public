// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.group.cube;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arms.ArmPosition;
import frc.robot.commands.wrist.WristPosition;

public class CubeScoreMiddle extends SequentialCommandGroup {
  public CubeScoreMiddle() {
    addCommands(
      new ArmPosition(Constants.STATE.SCORE_CUBE_MIDDLE.arms).deadlineWith(
        new WristPosition(Constants.STATE.SCORE_CUBE_MIDDLE.wrist)
      )
    );
  }
}
