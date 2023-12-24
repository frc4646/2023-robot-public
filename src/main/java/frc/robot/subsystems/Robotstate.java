// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.GAMEPIECE_MODE;
import frc.team4646.SmartSubsystem;

public class Robotstate extends SmartSubsystem {
  private final double GRID_X = Units.inchesToMeters(4.6875);
  private final double GRID_TO_GAMEPIECE_X = Units.inchesToMeters(224.0);
  private final double GAMEPIECE_TO_CENTER_X = Units.inchesToMeters(47.36);
  private final double FIELD_COMMUNITY_X = Units.inchesToMeters(118.25);
  private final double FIELD_NEUTRAL_ZONE_X = GRID_X + GRID_TO_GAMEPIECE_X * 2.0 + GAMEPIECE_TO_CENTER_X * 2.0;

  private final Vision visionF = RobotContainer.FRONT_VISION;
  private final SwerveDriveSubsystem drive = RobotContainer.SWERVE;

  private GAMEPIECE_MODE mode = GAMEPIECE_MODE.CONE;
  private boolean isEnabled;
  private final double ALIGNED_THRESHOLD = 1.5;

  public Robotstate() {
    // Shuffleboard.getTab(Constants.COMP_DASH_NAME).addBoolean("Mode", () -> isConeMode())
    // .withProperties(Map.of("Color when true", Constants.DIAGNOSTICS.MODE_CONE.color.getHex(),
    //                        "Color when false", Constants.DIAGNOSTICS.MODE_CUBE.color.getHex()))
    // .withPosition(0, 0)
    // .withSize(2, 2);
    isEnabled = false;
  }

  public void setMode(GAMEPIECE_MODE mode) {
    this.mode = mode;
  }

  public boolean isConeMode() {
    return mode == GAMEPIECE_MODE.CONE;
  }

  public boolean isCubeMode() {
    return mode == GAMEPIECE_MODE.CUBE;
  }

  public boolean isTransportModeWanted() {
    return drive.getAveragedSpeedMps() > Constants.SWERVE_DRIVE.TRANSPORT_SPEED_MPS;
  }

  public boolean isRobotNearDriverStation(double metersAwayMax) {
    return drive.getOdometry().getEstimatedPosition().getX() < metersAwayMax;
  }

  public boolean inPoleScoringPosition() {
    return visionF.foundVisionTape() &&
    Math.abs(visionF.getVisionTapePosition().yOffset) < ALIGNED_THRESHOLD &&
    Math.abs(visionF.getVisionTapePosition().xOffset) < ALIGNED_THRESHOLD;
  }

  public boolean isRobotInNeutralZone() {
    return !isRobotInCommunity() && !isRobotInLoadingStation();
  }

  public boolean isRobotInLoadingStation() {
    return drive.getField().getRobotPose().getX() > FIELD_NEUTRAL_ZONE_X;
  }

  public boolean isRobotInCommunity() {
    return drive.getField().getRobotPose().getX() < FIELD_COMMUNITY_X;
  }
  
  public boolean isDisabled() {
    return !isEnabled;
  }

  public boolean isBlueAlliance() {
    return DriverStation.getAlliance() == Alliance.Blue;
  }
  
  @Override
  public void onEnable(boolean isAutonomous) {
    isEnabled = true;
  }

  @Override
  public void onDisable() {
    isEnabled = false;
  }
}
