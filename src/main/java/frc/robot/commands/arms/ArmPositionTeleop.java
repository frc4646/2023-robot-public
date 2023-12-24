// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.configuration.OperatorControls;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Robotstate;

public class ArmPositionTeleop extends CommandBase {
  private final Robotstate state = RobotContainer.STATE;
  private final ArmSubsystem arms = RobotContainer.ARMS;
  // private final SwerveDriveSubsystem drive = RobotContainer.SWERVE;
  private final OperatorControls controls = RobotContainer.operator;

  public ArmPositionTeleop() {
    addRequirements(arms);
  }

  @Override
  public void execute() {
    double armsMainControls = controls.armsMain();
    double armsSecondaryControls = controls.armsSecondary();
    
    final boolean isTrimWanted = Math.abs(armsMainControls) > 0.0 || Math.abs(armsSecondaryControls) > 0.0;
    final boolean isOverrideWanted = controls.isOverridePressed();

    arms.setSoftLimitSwitchEnabled(!isOverrideWanted);

    //if (!arms.isHomedPreviously() || isOverrideWanted) {
    if (isOverrideWanted || isTrimWanted) {
      //arms.setHomeLocationIfHome();
      // SmartDashboard.putNumber("ControlsMain", armsMainControls);
      // SmartDashboard.putNumber("ControlsSec", armsSecondaryControls);
      arms.setTargetVoltage(armsMainControls * Constants.ARMS.TRIM.VOLTS_PER_STICK_MAIN,
                            armsSecondaryControls * Constants.ARMS.TRIM.VOLTS_PER_STICK_SECONDARY);
    } 
    // else if (isTrimWanted) {
    //   ArmAngles setpoint = arms.getAnglesRaw();
    //   SmartDashboard.putNumber("CurrentMain", setpoint.mainDegrees);
    //   SmartDashboard.putNumber("CurrentSec", setpoint.secondaryDegrees);
    //   setpoint = setpoint.trim(controls.armsMain() * Constants.ARMS.TRIM.DEGREES_PER_STICK_MAIN,
    //                            controls.armsSecondary() * Constants.ARMS.TRIM.DEGREES_PER_STICK_SECONDARY);
    //   SmartDashboard.putNumber("SetpointMain", setpoint.mainDegrees);
    //   SmartDashboard.putNumber("SetpointSec", setpoint.secondaryDegrees);
    //   arms.setTargetPositionRaw(setpoint);
    // } 
    else if (state.isTransportModeWanted()) {
      arms.setTargetPositionRaw(Constants.STATE.TRANSPORT.arms);
    } 
    else {
      arms.setTargetVoltage(0.0, 0.0);
    }
  }
}