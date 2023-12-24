// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.group.cone;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arms.ArmPosition;
import frc.robot.commands.arms.IfArmSecondaryBelow;
import frc.robot.commands.wrist.WristPosition;

public class ConeScoreTop extends SequentialCommandGroup {
  public ConeScoreTop() {
    this.addCommands(
      new IfArmSecondaryBelow(
        new ArmPosition(Constants.STATE.SCORE_CONE_TOP_PRE.arms).deadlineWith(
          new WristPosition(Constants.STATE.SCORE_CONE_TOP_PRE.wrist),
          new ConeIn(0.5)
        ).withTimeout(1.5),
        Constants.STATE.SCORE_CONE_TOP_PRE.arms.secondaryDegrees
      ),
      new ArmPosition(Constants.STATE.SCORE_CONE_TOP.arms).deadlineWith(
        new WristPosition(Constants.STATE.SCORE_CONE_TOP.wrist),
        new ConeIn(0.5)
      )
      // new WaitUntilCommand(() -> RobotContainer.STATE.inPoleScoringPosition()),
      // new WristPosition(Constants.STATE.SCORE_CONE_TOP_END.wrist),
      // new ConeEject(),
      // new WristPosition(Constants.STATE.SCORE_CONE_TOP.wrist)
    );
  }
}
