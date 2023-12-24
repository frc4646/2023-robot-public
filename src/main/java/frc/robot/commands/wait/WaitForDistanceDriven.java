package frc.robot.commands.wait;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class WaitForDistanceDriven extends CommandBase {
  private final SwerveDriveSubsystem drive = RobotContainer.SWERVE;
  private final double distanceRelativeWanted;
  private Translation2d initial = new Translation2d(0.0, 0.0);

  public WaitForDistanceDriven(double distance) {
    this.distanceRelativeWanted = distance;
  }

  @Override
  public void initialize() {
    initial = drive.getPose().getTranslation();
  }

  @Override
  public boolean isFinished() {
    return drive.getPose().getTranslation().getDistance(initial) >= distanceRelativeWanted;
  }
}
