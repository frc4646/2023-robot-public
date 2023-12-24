// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveOpenLoop;
import frc.robot.commands.group.TransportMode;
import frc.robot.commands.group.cone.ConeGroundPickUp;
import frc.robot.commands.group.cube.CubePickUp;
import frc.robot.util.GAMEPIECE_MODE;
import frc.robot.util.STARTING_LOCATION;

public class ModeCenter extends SequentialCommandGroup {
  public ModeCenter(boolean over, GAMEPIECE_MODE gamepiece) {
    if(over) {
      if(gamepiece == GAMEPIECE_MODE.CUBE) {
        addCommands(
          new StartOfAuto(STARTING_LOCATION.CENTER, GAMEPIECE_MODE.CUBE),
          Paths.getDriveOverChargingStation(STARTING_LOCATION.CENTER),
          new ConditionalDriveTrajectory(Paths.TRAJECTORY_BLUE_2_TO_GAMEPIECE, Paths.TRAJECTORY_RED_2_TO_GAMEPIECE).deadlineWith(new CubePickUp()),
          new ConditionalDriveTrajectory(Paths.TRAJECTORY_BLUE_2_BACK_TO_CHARGE_STATION, Paths.TRAJECTORY_RED_2_BACK_TO_CHARGE_STATION).deadlineWith(new TransportMode()),
          new DriveOpenLoop(new Translation2d(0.0, 0.0)).withTimeout(0.2),
          new ChargingStationClimb()
        );
      } else {
        addCommands(
          new StartOfAuto(STARTING_LOCATION.CENTER, GAMEPIECE_MODE.CONE),
          Paths.getDriveOverChargingStation(STARTING_LOCATION.CENTER),
          new ConditionalDriveTrajectory(Paths.TRAJECTORY_BLUE_2_TO_GAMEPIECE, Paths.TRAJECTORY_RED_2_TO_GAMEPIECE).deadlineWith(new ConeGroundPickUp()),
          new ConditionalDriveTrajectory(Paths.TRAJECTORY_BLUE_2_BACK_TO_CHARGE_STATION, Paths.TRAJECTORY_RED_2_BACK_TO_CHARGE_STATION).deadlineWith(new TransportMode()),
          new DriveOpenLoop(new Translation2d(0.0, 0.0)).withTimeout(0.2),
          new ChargingStationClimb()
        );
      }
    }
    else{
      addCommands(
        new StartOfAuto(STARTING_LOCATION.CENTER, GAMEPIECE_MODE.NONE),
        //Paths.getDriveToChargingStation(STARTING_LOCATION.CENTER),
        new ChargingStationClimb()
      );
    }  
  }
}
