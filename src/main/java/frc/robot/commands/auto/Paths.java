// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveTrajectory;
import frc.robot.util.STARTING_LOCATION;

/** Units are in meters */
public class Paths {
  public static final double GAMEPIECE_X_METERS = Units.inchesToMeters(18.0 * 12.0 + 8.0);

  private static final PathPoint 
    POINT_BLUE_1_START = new PathPoint(new Translation2d(1.86, 4.44), Rotation2d.fromDegrees(0.0)),
    POINT_BLUE_1_AVOID_CHARGING_STATION = new PathPoint(new Translation2d(3.2, 4.65), Rotation2d.fromDegrees(0.0)),
    POINT_BLUE_1_AVOID_CHARGING_STATION_BACK = new PathPoint(new Translation2d(2.7, 4.65), Rotation2d.fromDegrees(0.0)),
    POINT_BLUE_1_PRE_GAMEPIECE =  new PathPoint(new Translation2d(5.5, 4.54), Rotation2d.fromDegrees(0.0)),
    POINT_BLUE_1_GAMEPIECE =  new PathPoint(new Translation2d(6.75, 4.54), Rotation2d.fromDegrees(0.0)),
    POINT_BLUE_1_GAMEPIECE_TURN_RIGHT =  new PathPoint(new Translation2d(6.5, 4.2), Rotation2d.fromDegrees(230.0), Rotation2d.fromDegrees(270.0)),
    POINT_BLUE_1_CHARGING_STATION =  new PathPoint(new Translation2d(5.18, 3.0), Rotation2d.fromDegrees(180.0)),
    POINT_BLUE_1_GRID = new PathPoint(new Translation2d(1.86, 3.92), Rotation2d.fromDegrees(180.0)),

    POINT_BLUE_2_START = new PathPoint(new Translation2d(1.86, 2.78), Rotation2d.fromDegrees(0.0)),
    POINT_BLUE_2_CHARGING_STATION = new PathPoint(new Translation2d(2.31, 2.78), Rotation2d.fromDegrees(0.0)),
    POINT_BLUE_2_OVER_CHARGING_STATION = new PathPoint(new Translation2d(6.152, 2.78), Rotation2d.fromDegrees(0.0)),
    POINT_BLUE_2_GAMEPIECE = new PathPoint(new Translation2d(7.4, 2.42), Rotation2d.fromDegrees(-25.0)), //2.42
    POINT_BLUE_2_RETURN_TO_CHARGING_STATION = new PathPoint(new Translation2d(6.2, 2.78), Rotation2d.fromDegrees(180.0)),
    
    POINT_BLUE_3_START = new PathPoint(new Translation2d(1.86, 1.12), Rotation2d.fromDegrees(0.0)),
    POINT_BLUE_3_AVOID_CHARGING_STATION = new PathPoint(new Translation2d(2.8, .92), Rotation2d.fromDegrees(0.0)),
    POINT_BLUE_3_PRE_GAMEPIECE = new PathPoint(new Translation2d(5.5, .942), Rotation2d.fromDegrees(0.0)),
    POINT_BLUE_3_GAMEPIECE = new PathPoint(new Translation2d(6.67, .942), Rotation2d.fromDegrees(0.0)),
    POINT_BLUE_3_GAMEPIECE_TURN_LEFT = new PathPoint(new Translation2d(6.3, 1.5), Rotation2d.fromDegrees(130.0), Rotation2d.fromDegrees(270.0)),
    POINT_BLUE_3_CHARGING_STATION = new PathPoint(new Translation2d(5.18, 2.5), Rotation2d.fromDegrees(180.0)),
    POINT_BLUE_3_GRID = new PathPoint(new Translation2d(1.5, 1.12), Rotation2d.fromDegrees(180.0));

  public static final PathPlannerTrajectory
    // BLUE
    TRAJECTORY_BLUE_1_TO_GAMEPIECE_PRE = PathPlanner.generatePath(Constants.SWERVE_DRIVE.PATH_CONSTRAINTS, POINT_BLUE_1_START, POINT_BLUE_1_AVOID_CHARGING_STATION, POINT_BLUE_1_PRE_GAMEPIECE),
    TRAJECTORY_BLUE_1_TO_GAMEPIECE_FINAL = PathPlanner.generatePath(DriveTrajectory.PathConstraintsScaled(0.25), POINT_BLUE_1_PRE_GAMEPIECE, POINT_BLUE_1_GAMEPIECE),
    TRAJECTORY_BLUE_1_CHARGING_STATION = PathPlanner.generatePath(Constants.SWERVE_DRIVE.PATH_CONSTRAINTS, POINT_BLUE_1_GAMEPIECE, POINT_BLUE_1_GAMEPIECE_TURN_RIGHT, POINT_BLUE_1_CHARGING_STATION),
    TRAJECTORY_BLUE_1_GRID = PathPlanner.generatePath(Constants.SWERVE_DRIVE.PATH_CONSTRAINTS, POINT_BLUE_1_GAMEPIECE, POINT_BLUE_1_AVOID_CHARGING_STATION_BACK, POINT_BLUE_1_GRID),

    TRAJECTORY_BLUE_2_HOME_TO_CHARGING_STATION = PathPlanner.generatePath(Constants.SWERVE_DRIVE.PATH_CONSTRAINTS, POINT_BLUE_2_START, POINT_BLUE_2_CHARGING_STATION),
    TRAJECTORY_BLUE_2_HOME_OVER_CHARGING_STATION = PathPlanner.generatePath(DriveTrajectory.PathConstraintsScaled(0.5), POINT_BLUE_2_START, POINT_BLUE_2_OVER_CHARGING_STATION),
    TRAJECTORY_BLUE_2_TO_GAMEPIECE = PathPlanner.generatePath(DriveTrajectory.PathConstraintsScaled(0.5), POINT_BLUE_2_OVER_CHARGING_STATION, POINT_BLUE_2_GAMEPIECE),
    TRAJECTORY_BLUE_2_BACK_TO_CHARGE_STATION = PathPlanner.generatePath(DriveTrajectory.PathConstraintsScaled(0.5), POINT_BLUE_2_GAMEPIECE, POINT_BLUE_2_RETURN_TO_CHARGING_STATION),

    TRAJECTORY_BLUE_3_TO_GAMEPIECE_PRE = PathPlanner.generatePath(DriveTrajectory.PathConstraintsScaled(0.50), POINT_BLUE_3_START, POINT_BLUE_3_AVOID_CHARGING_STATION, POINT_BLUE_3_PRE_GAMEPIECE),
    TRAJECTORY_BLUE_3_TO_GAMEPIECE_FINAL = PathPlanner.generatePath(DriveTrajectory.PathConstraintsScaled(0.25), POINT_BLUE_3_PRE_GAMEPIECE, POINT_BLUE_3_GAMEPIECE),
    TRAJECTORY_BLUE_3_GAMEPIECE_TO_CHARGING_STATION = PathPlanner.generatePath(Constants.SWERVE_DRIVE.PATH_CONSTRAINTS, POINT_BLUE_3_GAMEPIECE, POINT_BLUE_3_GAMEPIECE_TURN_LEFT, POINT_BLUE_3_CHARGING_STATION),
    TRAJECTORY_BLUE_3_GRID = PathPlanner.generatePath(Constants.SWERVE_DRIVE.PATH_CONSTRAINTS, POINT_BLUE_3_GAMEPIECE, POINT_BLUE_3_AVOID_CHARGING_STATION, POINT_BLUE_3_GRID),
    
    // RED
    TRAJECTORY_RED_1_TO_GAMEPIECE_PRE = PathPlannerTrajectory.transformTrajectoryForAlliance(TRAJECTORY_BLUE_1_TO_GAMEPIECE_PRE, Alliance.Red),
    TRAJECTORY_RED_1_TO_GAMEPIECE_FINAL = PathPlannerTrajectory.transformTrajectoryForAlliance(TRAJECTORY_BLUE_1_TO_GAMEPIECE_FINAL, Alliance.Red),
    TRAJECTORY_RED_1_CHARGING_STATION = PathPlannerTrajectory.transformTrajectoryForAlliance(TRAJECTORY_BLUE_1_CHARGING_STATION, Alliance.Red),
    TRAJECTORY_RED_1_GRID = PathPlannerTrajectory.transformTrajectoryForAlliance(TRAJECTORY_BLUE_1_GRID, Alliance.Red),

    TRAJECTORY_RED_2_HOME_TO_CHARGING_STATION = PathPlannerTrajectory.transformTrajectoryForAlliance(TRAJECTORY_BLUE_2_HOME_TO_CHARGING_STATION, Alliance.Red),
    TRAJECTORY_RED_2_HOME_OVER_CHARGING_STATION = PathPlannerTrajectory.transformTrajectoryForAlliance(TRAJECTORY_BLUE_2_HOME_OVER_CHARGING_STATION, Alliance.Red),
    TRAJECTORY_RED_2_TO_GAMEPIECE = PathPlannerTrajectory.transformTrajectoryForAlliance(TRAJECTORY_BLUE_2_TO_GAMEPIECE, Alliance.Red),
    TRAJECTORY_RED_2_BACK_TO_CHARGE_STATION = PathPlannerTrajectory.transformTrajectoryForAlliance(TRAJECTORY_BLUE_2_BACK_TO_CHARGE_STATION, Alliance.Red),

    TRAJECTORY_RED_3_TO_GAMEPIECE_PRE = PathPlannerTrajectory.transformTrajectoryForAlliance(TRAJECTORY_BLUE_3_TO_GAMEPIECE_PRE, Alliance.Red),
    TRAJECTORY_RED_3_TO_GAMEPIECE_FINAL = PathPlannerTrajectory.transformTrajectoryForAlliance(TRAJECTORY_BLUE_3_TO_GAMEPIECE_FINAL, Alliance.Red),
    TRAJECTORY_RED_3_GAMEPIECE_TO_CHARGING_STATION = PathPlannerTrajectory.transformTrajectoryForAlliance(TRAJECTORY_BLUE_3_GAMEPIECE_TO_CHARGING_STATION, Alliance.Red),
    TRAJECTORY_RED_3_GRID = PathPlannerTrajectory.transformTrajectoryForAlliance(TRAJECTORY_BLUE_3_GRID, Alliance.Red);

  static {
    final Field2d field = RobotContainer.SWERVE.getField();
    field.getObject("trajBlue1APreGamepiece").setTrajectory(Paths.TRAJECTORY_BLUE_1_TO_GAMEPIECE_PRE);
    field.getObject("trajBlue1A").setTrajectory(Paths.TRAJECTORY_BLUE_1_TO_GAMEPIECE_FINAL);
    field.getObject("trajBlue1B").setTrajectory(Paths.TRAJECTORY_BLUE_1_CHARGING_STATION);
    field.getObject("trajBlue1C").setTrajectory(Paths.TRAJECTORY_BLUE_1_GRID);
    
    field.getObject("trajBlue2").setTrajectory(Paths.TRAJECTORY_BLUE_2_HOME_TO_CHARGING_STATION);
    field.getObject("trajBlue2Over").setTrajectory(Paths.TRAJECTORY_BLUE_2_HOME_OVER_CHARGING_STATION);
    field.getObject("trajBlue2Pickup").setTrajectory(Paths.TRAJECTORY_BLUE_2_TO_GAMEPIECE);

    field.getObject("trajBlue3APreGamepiece").setTrajectory(Paths.TRAJECTORY_BLUE_3_TO_GAMEPIECE_PRE);
    field.getObject("trajBlue3A").setTrajectory(Paths.TRAJECTORY_BLUE_3_TO_GAMEPIECE_FINAL);
    field.getObject("trajBlue3B").setTrajectory(Paths.TRAJECTORY_BLUE_3_GAMEPIECE_TO_CHARGING_STATION);
    field.getObject("trajBlue3C").setTrajectory(Paths.TRAJECTORY_BLUE_3_GRID);

    // field.getObject("trajRed1APreGamepiece").setTrajectory(Paths.TRAJECTORY_RED_1_TO_GAMEPIECE);
    // field.getObject("trajRed1A").setTrajectory(Paths.TRAJECTORY_RED_1_TO_GAMEPIECE_FINAL);
    // field.getObject("trajRed1B").setTrajectory(Paths.TRAJECTORY_RED_1_CHARGING_STATION);
    // field.getObject("trajRed1C").setTrajectory(Paths.TRAJECTORY_RED_1_GRID);
    
    // field.getObject("trajRed2").setTrajectory(Paths.TRAJECTORY_RED_2_HOME_TO_CHARGING_STATION);
    // field.getObject("trajRed2Over").setTrajectory(Paths.TRAJECTORY_RED_2_HOME_OVER_CHARGING_STATION);
    // field.getObject("trajRed2Pickup").setTrajectory(Paths.TRAJECTORY_RED_2_TO_GAMEPIECE);
    
    // field.getObject("trajRed3APreGamepiece").setTrajectory(Paths.TRAJECTORY_RED_3_TO_GAMEPIECE_PRE);
    // field.getObject("trajRed3A").setTrajectory(Paths.TRAJECTORY_RED_3_TO_GAMEPIECE_FINAL);
    // field.getObject("trajRed3B").setTrajectory(Paths.TRAJECTORY_RED_3_GAMEPIECE_TO_CHARGING_STATION);
    // field.getObject("trajRed3C").setTrajectory(Paths.TRAJECTORY_RED_3_GRID);
  }
  
  public static ConditionalSetPose getSetPose(STARTING_LOCATION location) {
    switch (location) {
      case CENTER:
        return new ConditionalSetPose(Paths.TRAJECTORY_BLUE_2_HOME_TO_CHARGING_STATION, Paths.TRAJECTORY_RED_2_HOME_TO_CHARGING_STATION);
      case LOADING_ZONE:
        return new ConditionalSetPose(Paths.TRAJECTORY_BLUE_1_TO_GAMEPIECE_PRE, Paths.TRAJECTORY_RED_1_TO_GAMEPIECE_PRE);
      case CABLE_PROTECTOR:
      default:
        return new ConditionalSetPose(Paths.TRAJECTORY_BLUE_3_TO_GAMEPIECE_PRE, Paths.TRAJECTORY_RED_3_TO_GAMEPIECE_PRE);
    }
  }

  public static ConditionalDriveTrajectory getDriveToGamepiece(STARTING_LOCATION location) {
    switch (location) {
      case LOADING_ZONE:
        return (new ConditionalDriveTrajectory(Paths.TRAJECTORY_BLUE_1_TO_GAMEPIECE_PRE, Paths.TRAJECTORY_RED_1_TO_GAMEPIECE_PRE));
      case CABLE_PROTECTOR:
      default:
        return new ConditionalDriveTrajectory(Paths.TRAJECTORY_BLUE_3_TO_GAMEPIECE_PRE, Paths.TRAJECTORY_RED_3_TO_GAMEPIECE_PRE);
    }
  }

  public static ConditionalDriveTrajectory getDrivePickUpGamePiece(STARTING_LOCATION location) {
    switch (location) {
      case LOADING_ZONE:
        return (new ConditionalDriveTrajectory(Paths.TRAJECTORY_BLUE_1_TO_GAMEPIECE_FINAL, Paths.TRAJECTORY_RED_1_TO_GAMEPIECE_PRE));
      case CABLE_PROTECTOR:
      default:
        return new ConditionalDriveTrajectory(Paths.TRAJECTORY_BLUE_3_TO_GAMEPIECE_FINAL, Paths.TRAJECTORY_RED_3_TO_GAMEPIECE_FINAL);
    }
  }

  public static ConditionalDriveTrajectory getDriveBackToGrid(STARTING_LOCATION location) {
    switch (location) {
      case LOADING_ZONE:
        return new ConditionalDriveTrajectory(Paths.TRAJECTORY_BLUE_1_GRID, Paths.TRAJECTORY_RED_1_GRID);
      case CABLE_PROTECTOR:
      default:
      return new ConditionalDriveTrajectory(Paths.TRAJECTORY_BLUE_3_GRID, Paths.TRAJECTORY_RED_3_GRID);
    }
  }
  


  public static ConditionalDriveTrajectory getDriveOverChargingStation(STARTING_LOCATION location) {
    switch (location) {
      case CENTER:
        return new ConditionalDriveTrajectory(Paths.TRAJECTORY_BLUE_2_HOME_OVER_CHARGING_STATION,Paths.TRAJECTORY_RED_2_HOME_OVER_CHARGING_STATION);
      case LOADING_ZONE:
        return new ConditionalDriveTrajectory(Paths.TRAJECTORY_BLUE_1_CHARGING_STATION, Paths.TRAJECTORY_RED_1_CHARGING_STATION);
      case CABLE_PROTECTOR:
      default:
        return new ConditionalDriveTrajectory(Paths.TRAJECTORY_BLUE_3_GAMEPIECE_TO_CHARGING_STATION, Paths.TRAJECTORY_RED_3_GAMEPIECE_TO_CHARGING_STATION);
    }
  }

  public static ConditionalDriveTrajectory getDriveToChargingStation(STARTING_LOCATION location) {
    switch (location) {
      case CENTER:
        return new ConditionalDriveTrajectory(Paths.TRAJECTORY_BLUE_2_HOME_TO_CHARGING_STATION,Paths.TRAJECTORY_RED_2_HOME_TO_CHARGING_STATION);
      case LOADING_ZONE:
        return new ConditionalDriveTrajectory(Paths.TRAJECTORY_BLUE_1_CHARGING_STATION, Paths.TRAJECTORY_RED_1_CHARGING_STATION);
      case CABLE_PROTECTOR:
      default:
        return new ConditionalDriveTrajectory(Paths.TRAJECTORY_BLUE_3_GAMEPIECE_TO_CHARGING_STATION, Paths.TRAJECTORY_RED_3_GAMEPIECE_TO_CHARGING_STATION);
    }
  }
} 
