// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveOpenLoop;
import frc.robot.commands.group.cone.ConeGroundPickUp;
import frc.robot.commands.group.cube.CubePickUp;
import frc.robot.commands.intake.IntakeInstant;
import frc.robot.util.GAMEPIECE_MODE;
import frc.robot.util.STARTING_LOCATION;

public class NeutralZoneGamepiece extends SequentialCommandGroup {
  public NeutralZoneGamepiece(STARTING_LOCATION location, GAMEPIECE_MODE gamepieceToPickUp) {
    if (gamepieceToPickUp == GAMEPIECE_MODE.CUBE) {
      addCommands(
        new CubePickUp().withTimeout(5.5).deadlineWith(
          Paths.getDriveToGamepiece(location).andThen(
            Paths.getDrivePickUpGamePiece(location)
          )
        ),
        new DriveOpenLoop(new Translation2d(0.0, 0.0)).withTimeout(0.1).deadlineWith(
          new IntakeInstant()
        )
      );
    } else {
      addCommands(
        new ConeGroundPickUp().withTimeout(4.0).deadlineWith(
          Paths.getDriveToGamepiece(location).andThen(
            Paths.getDrivePickUpGamePiece(location)
          )
        ),
        new DriveOpenLoop(new Translation2d(0.0, 0.0)).withTimeout(0.3).deadlineWith(
          new IntakeInstant()
        )
        // new ParallelCommandGroup(
        //   new DriveTrajectoryConditional(trajectoryBlue, trajectoryRed).andThen(
        //     new DriveOpenLoop(new Translation2d(0.0, 0.0)).withTimeout(0.2)
        //   ),      
        //   new WingsAuto().andThen(
        //     new ConePickUpCommon().withTimeout(5.0)  // Timeout so command always finishes in auto
        //   )
        // ),
        // Path stop once cone in robot so that robot doesn't drive backwards during end of path
        // new WingsAuto().deadlineWith(
        //   Paths.getDriveToGamepiece(location)
        // ),
        // new ConePickUpCommon().withTimeout(5.0).deadlineWith(  // Timeout so command always finishes in auto
        //   new DriveOpenLoop(new Translation2d(0.4, 0.0)).withTimeout(0.4).andThen(
        //     new DriveOpenLoop(new Translation2d(0.0, 0.0)).withTimeout(0.2)
        //   )
        // ),
        // new WingsInstant(false)  // Prep for climbing charging station
      );
    }
  }
}
