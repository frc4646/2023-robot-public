// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveOpenLoop;
import frc.robot.commands.drive.DriveTurnToAngle;
import frc.robot.commands.wait.WaitForDistanceX;
import frc.robot.commands.wait.WaitForPitch.DIRECTION;

public class ChargingStationCross extends SequentialCommandGroup {
  private final double
    CLIMB_MPS = 0.6, CLIME_TIMEOUT = 8.0, CLIMB_DEGREES = 12.0,
    FALL_MPS = 0.6, FALL_TIMEOUT = 3.0;

  public ChargingStationCross() {
    addCommands(
      new WaitForDistanceX(Paths.GAMEPIECE_X_METERS - 1.0).raceWith(  // prevent accidently driving into neutral zone
        new SequentialCommandGroup(
          new DrivePitch(CLIMB_MPS, DIRECTION.ABOVE, CLIMB_DEGREES, 3, CLIME_TIMEOUT),
          new DriveOpenLoop(new Translation2d(FALL_MPS, 0.0)).withTimeout(FALL_TIMEOUT)
        )
      ),
      new DriveTurnToAngle(180.0, 2.0)
    );
  }
}
