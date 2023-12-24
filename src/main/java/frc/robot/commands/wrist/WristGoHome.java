package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.WristSubsystem;
import frc.team4646.StabilityCounter;

public class WristGoHome extends CommandBase {
  private final WristSubsystem subsystem = RobotContainer.WRIST;
  
  private final StabilityCounter counterAtHome;

  public WristGoHome() {
    addRequirements(subsystem);
    counterAtHome = new StabilityCounter(10);

  }

  @Override
  public void initialize() {
    subsystem.setSoftLimitSwitchEnabled(false);
    counterAtHome.reset();
  }
  @Override
  public void execute() {
    //subsystem.setHomeLocationIfHome();
    subsystem.setVoltage(0.5);
    counterAtHome.calculate(subsystem.isHome());
  }

  @Override
  public boolean isFinished() {
    return counterAtHome.isStable();
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.setSoftLimitSwitchEnabled(true);
  }
}
