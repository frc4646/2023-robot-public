// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveOpenLoop;
import frc.robot.commands.group.IfHasGamepiece;
import frc.robot.commands.group.TransportMode;
import frc.robot.commands.group.cone.ConeOut;
import frc.robot.commands.group.cone.ConeScoreTop;
import frc.robot.commands.group.cube.CubeOut;
import frc.robot.util.GAMEPIECE_MODE;
import frc.robot.util.STARTING_LOCATION;

public class ModeSide extends SequentialCommandGroup {
  public ModeSide(STARTING_LOCATION location, GAMEPIECE_MODE gamepieceToPickUp, boolean balance) {
    addCommands(
      new StartOfAuto(location, gamepieceToPickUp),
      new NeutralZoneGamepiece(location, gamepieceToPickUp)
    );

    if (balance) {
      addCommands(
        Paths.getDriveToChargingStation(location).alongWith(
          new TransportMode().withTimeout(2.0)
        ),
        new ChargingStationClimb()
      );
    } else {
      addCommands(
        new IfHasGamepiece(
          Paths.getDriveBackToGrid(location).deadlineWith(
            new TransportMode()
          ).andThen(
            new DriveOpenLoop(new Translation2d(0.0, 0.0)).withTimeout(0.2).andThen(
                gamepieceToPickUp == GAMEPIECE_MODE.CUBE ? new CubeOut() : 
              new ConeScoreTop().andThen(
                new WaitCommand(0.2),
              new ConeOut().withTimeout(0.3)
              ).andThen(
                new TransportMode()
              )
              

            )
          )
        )
      );
    }
  }
}
