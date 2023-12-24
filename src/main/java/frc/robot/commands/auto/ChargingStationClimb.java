// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveLock;
import frc.robot.commands.drive.DriveOpenLoop;
import frc.robot.commands.wait.WaitForPitch.DIRECTION;

public class ChargingStationClimb extends ProxyCommand {
  public static double
    // APPROACH_MPS = 0.2, APPROACH_TIMEOUT = 6.0, APPROACH_DEGREES = 3.0,
    CLIMB_MPS = 1.8, CLIMB_TIMEOUT = 8.0, CLIMB_DEGREES = 12.0,
    FALL_MPS = 0.4, FALL_TIMEOUT = 6.0, FALL_DEGREES = 11.0,
    REVERSE_MPS = -0.6, REVERSE_TIMEOUT = 0.32;

  public ChargingStationClimb() {
    super(ChargingStationClimb::select);
  }

  public static Command select() {
    DriverStation.reportWarning("" + CLIMB_MPS, false);
    return new SequentialCommandGroup(
      // new WaitForDistanceX(Paths.GAMEPIECE_X_METERS).raceWith(  // prevent accidently driving into neutral zone
      // new SequentialCommandGroup(
      // new DrivePitch(APPROACH_MPS, DIRECTION.ABOVE, APPROACH_DEGREES, 2, APPROACH_TIMEOUT),
      new DrivePitch(CLIMB_MPS, DIRECTION.ABOVE, CLIMB_DEGREES, 3, CLIMB_TIMEOUT),
      new DrivePitch(FALL_MPS, DIRECTION.BELOW, FALL_DEGREES, 2, FALL_TIMEOUT),
      new DriveOpenLoop(new Translation2d(REVERSE_MPS, 0.0)).withTimeout(REVERSE_TIMEOUT),
      new DriveLock()
    );
  }

  public static void setClimbSpeed(double value) { CLIMB_MPS = value; }
  public static void setClimbDegrees(double value) { CLIMB_DEGREES = value; }
  public static void setClimbTimeout(double value) { CLIMB_TIMEOUT = value; }
  public static void setFallSpeed(double value) { FALL_MPS = value; }
  public static void setFallDegrees(double value) { FALL_DEGREES = value; }
  public static void setFallTimeout(double value) { FALL_TIMEOUT = value; }
  public static void setReverseSpeed(double value) { REVERSE_MPS = value; }
  public static void setReverseTimeout(double value) { REVERSE_TIMEOUT = value; }
}
