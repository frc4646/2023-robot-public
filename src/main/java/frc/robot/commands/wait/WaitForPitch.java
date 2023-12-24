package frc.robot.commands.wait;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.team4646.StabilityCounter;

public class WaitForPitch extends CommandBase {
  public static enum DIRECTION { ABOVE, BELOW };
  
  private final SwerveDriveSubsystem drive = RobotContainer.SWERVE;
  private final DIRECTION directionWanted;
  private final double pitchWanted;
  private final StabilityCounter stability;

  public WaitForPitch(DIRECTION direction, double pitch, int countsMustBeStable) {
    this.directionWanted = direction;
    this.pitchWanted = pitch;
    stability = new StabilityCounter(countsMustBeStable);
  }

  @Override
  public void initialize() {
    stability.reset();
  }

  @Override
  public void execute() {
    boolean isSampleCorrect;

    if (directionWanted == DIRECTION.ABOVE) {
      isSampleCorrect = drive.getPitch().getDegrees() > pitchWanted;
    } else {
      isSampleCorrect = drive.getPitch().getDegrees() < pitchWanted;
    }
    stability.calculate(isSampleCorrect);
  }
  
  @Override
  public boolean isFinished() {
    return stability.isStable();
  }
}
