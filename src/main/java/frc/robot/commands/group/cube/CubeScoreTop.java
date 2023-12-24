package frc.robot.commands.group.cube;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arms.ArmPosition;
import frc.robot.commands.arms.IfArmSecondaryBelow;
import frc.robot.commands.wrist.WristPosition;

public class CubeScoreTop extends SequentialCommandGroup {
    public CubeScoreTop() {
        addCommands(
          new IfArmSecondaryBelow(
            new ArmPosition(Constants.STATE.SCORE_CUBE_TOP_PRE.arms).deadlineWith(
              new WristPosition(Constants.STATE.SCORE_CUBE_TOP_PRE.wrist),
              new CubeIn(0.5)
            ).withTimeout(1.5),
            Constants.STATE.SCORE_CUBE_TOP_PRE.arms.secondaryDegrees
          ),
          new ArmPosition(Constants.STATE.SCORE_CUBE_TOP.arms).deadlineWith(
            new WristPosition(Constants.STATE.SCORE_CUBE_TOP.wrist)
          )
        );
    }    
}
