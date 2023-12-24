package frc.robot.commands.group.cone;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arms.ArmPosition;
import frc.robot.commands.intake.IntakeInstant;
import frc.robot.commands.wait.WaitForGamepiece;
import frc.robot.commands.wrist.WristPosition;

public class ConeGroundPickUp extends SequentialCommandGroup {

  public ConeGroundPickUp() {
    
    addCommands(
      new ArmPosition(Constants.STATE.PICKUP_CONE_GROUND_SLOW_DOWN.arms).deadlineWith(
        new WristPosition(Constants.STATE.PICKUP_CONE_GROUND_SLOW_DOWN.wrist)
      ).withTimeout(2.0).andThen(
        new WaitForGamepiece().deadlineWith(
          new ConeIn(),
          new ArmPosition(Constants.STATE.PICKUP_CONE_GROUND.arms).alongWith(
            new WristPosition(Constants.STATE.PICKUP_CONE_GROUND.wrist)
          )
        ),
        new ArmPosition(Constants.STATE.TRANSPORT.arms).alongWith(
          new WristPosition(Constants.STATE.TRANSPORT.wrist)
        ).deadlineWith(
          new ConeIn(.5)
        ),
        new IntakeInstant()
      )
    );
  }
}
