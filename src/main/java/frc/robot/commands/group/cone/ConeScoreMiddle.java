package frc.robot.commands.group.cone;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arms.ArmPosition;
import frc.robot.commands.wrist.WristPosition;

public class ConeScoreMiddle extends SequentialCommandGroup {
  public ConeScoreMiddle() {
    addCommands(
      new ArmPosition(Constants.STATE.SCORE_CONE_MIDDLE.arms).deadlineWith(
        new WristPosition(Constants.STATE.SCORE_CONE_MIDDLE.wrist)
      )
    );
  }
}
