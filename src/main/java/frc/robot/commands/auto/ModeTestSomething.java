// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class ModeTestSomething extends SequentialCommandGroup {
  double SAFE_MAIN = 104.0 - 5.0;
  double SAFE_SECONDARY = (Constants.ARMS.SECONDARY.DEGREES_MAX - Constants.ARMS.SECONDARY.DEGREES_MIN) / 2.0;
  double SAFE_WRIST = 155.0 - 5.0;
  double GROUND_DEPLOY = 0.0;
  double GROUND_STOW = 90.0;
  double INTAKE_ON = 0.0;
  double INTAKE_OFF = 0.0;

  PathConstraints PATH_SETTINGS = Constants.SWERVE_DRIVE.PATH_CONSTRAINTS;
  PathPoint POSE_START = new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0));
  PathPoint POSE_CIRCLE_LEFT = new PathPoint(new Translation2d(2.0, 0.5), Rotation2d.fromDegrees(0.0));
  PathPoint POSE_CIRCLE_TOP = new PathPoint(new Translation2d(2.5, 0.0), Rotation2d.fromDegrees(90.0));
  PathPlannerTrajectory PATH = PathPlanner.generatePath(
    PATH_SETTINGS,
    POSE_START,
    POSE_CIRCLE_LEFT,
    POSE_CIRCLE_TOP
  );
  
  public ModeTestSomething() {
    addCommands(
      new ChargingStationClimb()
      // // Test the ground intake picking up a cube while driving a path without stopping
      // new DriveSetPose(PATH.getInitialPose()),
      // new WristPosition(SAFE_WRIST),
      // new ArmPosition(SAFE_MAIN, SAFE_SECONDARY),
      // new NeutralZoneGamepiece(STARTING_LOCATION.LOADING_ZONE, GAMEPIECE_MODE.CUBE).deadlineWith(
      //   new GroundWristPosition(GROUND_DEPLOY).andThen(
      //     new GroundIntakeOpenLoop(INTAKE_ON, false),
      //     new WaitForGroundCone(),
      //     new GroundIntakeOpenLoop(INTAKE_OFF, false),
      //     new GroundWristPosition(GROUND_STOW)
      //   )
      // )
    );
  }
}
