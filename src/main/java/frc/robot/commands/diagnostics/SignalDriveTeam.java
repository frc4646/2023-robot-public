package frc.robot.commands.diagnostics;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
//import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.Robotstate;
//import frc.robot.subsystems.WristSubsystem;
import frc.team4646.Canifier;

public class SignalDriveTeam extends CommandBase {
  private final Diagnostics diagnostics = RobotContainer.DIAGNOSTICS;
  private final Robotstate state = RobotContainer.STATE;
  private final Canifier canifier = RobotContainer.CANIFIER;
  //private final ArmSubsystem arms = RobotContainer.ARMS;
  //private final WristSubsystem wrist = RobotContainer.WRIST;

  public SignalDriveTeam() {
    addRequirements(diagnostics);
  }

  @Override
  public void execute() {
    //if (!arms.isHomedPreviously() || !wrist.isHomedPreviously()) {
    //  diagnostics.setState(Constants.DIAGNOSTICS.FAULT_NOT_ZEROED);
    //}

    //else 
    if (state.isDisabled()) {
      diagnostics.setState(Constants.DIAGNOSTICS.NO_FAULT);
    }

    else if (state.isConeMode() ) {
      if(canifier.isConeDetected()) {
        diagnostics.setState(Constants.DIAGNOSTICS.MODE_CONE_CAPTURED);
      } else {
        diagnostics.setState(Constants.DIAGNOSTICS.MODE_CONE);
      }
    }

    else if (state.isCubeMode()) {
      if(canifier.isCubeDetected()) {
        diagnostics.setState(Constants.DIAGNOSTICS.MODE_CUBE_CAPTURED);
      } else {
        diagnostics.setState(Constants.DIAGNOSTICS.MODE_CUBE);
      }
    }
    
    else {
      diagnostics.setStateOkay();
    }

    RobotContainer.INFRASTRUCTURE.setArmsLEDs(true);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
