package frc.robot.commands.group.cube;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arms.ArmPosition;
import frc.robot.commands.intake.IntakeInstant;
import frc.robot.commands.wait.WaitForGamepiece;
import frc.robot.commands.wrist.WristPosition;

public class CubePickUp extends SequentialCommandGroup {
  public CubePickUp() {
    addCommands(
      new ArmPosition(Constants.STATE.PICKUP_CUBE_GROUND_SLOW_DOWN.arms).deadlineWith(
        new WristPosition(Constants.STATE.PICKUP_CUBE_GROUND_SLOW_DOWN.wrist)
      ).withTimeout(1.0).andThen(
        new WaitForGamepiece().deadlineWith(
          new CubeIn(),
          new ArmPosition(Constants.STATE.PICKUP_CUBE_GROUND.arms).alongWith(
            new WristPosition(Constants.STATE.PICKUP_CUBE_GROUND.wrist)
          )
        ),
        new IntakeInstant(),
        new ArmPosition(Constants.STATE.TRANSPORT.arms).alongWith(
          new WristPosition(Constants.STATE.TRANSPORT.wrist)
        )
      )
    );
  }
}
