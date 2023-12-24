// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.diagnostics.SetGamepieceMode;
import frc.robot.commands.stinger.CubeFlip;
import frc.robot.util.GAMEPIECE_MODE;
import frc.robot.util.STARTING_LOCATION;

public class StartOfAuto extends SequentialCommandGroup {
  public StartOfAuto(STARTING_LOCATION location, GAMEPIECE_MODE gamepieceToPickUp) {
    addCommands(
      Paths.getSetPose(location),
      new CubeFlip(true),
      new SetGamepieceMode(gamepieceToPickUp),
      new WaitCommand(0.35)
    );
    // if (gamepieceToPickUp == GAMEPIECE_MODE.CONE) {
    //   addCommands(new WingsInstant(true));
    // }
    addCommands(new WaitCommand(0.25));
  }
}
