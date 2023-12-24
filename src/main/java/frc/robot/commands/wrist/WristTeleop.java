// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.configuration.OperatorControls;
import frc.robot.subsystems.Robotstate;
import frc.robot.subsystems.WristSubsystem;

public class WristTeleop extends CommandBase {
  private final Robotstate state = RobotContainer.STATE;
  private final WristSubsystem wrist = RobotContainer.WRIST;
  // private final SwerveDriveSubsystem drive = RobotContainer.SWERVE;
  private final OperatorControls controls = RobotContainer.operator;

  public WristTeleop() {
    addRequirements(wrist);
  }

  @Override
  public void execute() {
    final boolean isTrimWanted = Math.abs(controls.wristOmega()) > 0.0;
    final boolean isOverrideWanted = controls.isOverridePressed();

    wrist.setSoftLimitSwitchEnabled(!isOverrideWanted);

    //if (!wrist.isHomedPreviously() || isOverrideWanted) {
    if (isOverrideWanted) {
      //wrist.setHomeLocationIfHome();
      wrist.setVoltage(controls.wristOmega() * Constants.WRIST.VOLTS_PER_STICK);
    } else if (isTrimWanted) {
      final double last = wrist.getAngleRaw().getDegrees();
      final double degrees = last + Constants.WRIST.DEGREES_PER_STICK * controls.wristOmega();
      wrist.setPosition(degrees);
    } else if (state.isTransportModeWanted()) {
      wrist.setPosition(Constants.WRIST.DEGREES_HOME);
    } else {
      wrist.setVoltage(0.0);
    }
  }
}
