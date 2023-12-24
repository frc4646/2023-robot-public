package frc.robot.commands.wait;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class WaitForDriveVelocity extends CommandBase {
  private final SwerveDriveSubsystem drive = RobotContainer.SWERVE;
  private final double speedWanted;

  public WaitForDriveVelocity(double metersPerSecond) {
    this.speedWanted = metersPerSecond;
  }

  @Override
  public boolean isFinished() {
    ChassisSpeeds chassisSpeeds = drive.getDesiredSpeeds();
    double speed = Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond,2) + Math.pow(chassisSpeeds.vyMetersPerSecond,2));
    return speed <= speedWanted;
  }
}
