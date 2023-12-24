package frc.robot.commands.drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class DriveTrajectory extends SequentialCommandGroup {
  private final SwerveDriveSubsystem swerve = RobotContainer.SWERVE;

  /**
   * Follow a trajectory, All units in meters.
   * <pre>
   * Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
   *    // Start at the origin facing the +X direction
   *    new Pose2d(0, 0, new Rotation2d(0)),
   *    // Pass through these two interior waypoints, making an 's' curve path
   *    List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
   *    // End 3 meters straight ahead of where we started, facing forward
   *    new Pose2d(3, 0, new Rotation2d(0)),
   *    Constants.SWERVE_DRIVE.TRAJECTORY_CONFIG);
   * </pre>
   */
  public DriveTrajectory(String name, PathPlannerTrajectory trajectory){
    this(trajectory);
    RobotContainer.SWERVE.getField().getObject(name).setTrajectory(trajectory);
  }

  public static PathConstraints PathConstraintsScaled(double factor)
  {
    return new PathConstraints(Constants.SWERVE_DRIVE.AUTO_MAX_SPEED_MPS*factor, Constants.SWERVE_DRIVE.AUTO_MAX_ACCEL_MPS*factor);
  }

  public DriveTrajectory(PathPlannerTrajectory trajectory){
    final ProfiledPIDController angleController = new ProfiledPIDController(
      Constants.SWERVE_DRIVE.P_ANGLE_CONTROLLER, 0.0, 0.0, 
      Constants.SWERVE_DRIVE.ANGLE_CONSTRAINTS
    );
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    final SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory,
      swerve::getPose,
      Constants.SWERVE_DRIVE.KINEMATICS,
      new PIDController(Constants.SWERVE_DRIVE.P_X_CONTROLLER, 0.0, 0.0),
      new PIDController(Constants.SWERVE_DRIVE.P_Y_CONTROLLER, 0.0, 0.0),
      angleController,
      // () -> new Rotation2d(), // optional time-sampled headings or constant heading
      swerve::setDesiredStates,
      swerve
    );
    addCommands(
      swerveControllerCommand
    );
  }
}