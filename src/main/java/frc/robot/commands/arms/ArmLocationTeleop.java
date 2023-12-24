// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.configuration.OperatorControls;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.IntakeLocation;

public class ArmLocationTeleop extends CommandBase {
  private final ArmSubsystem arms = RobotContainer.ARMS;
  private final SwerveDriveSubsystem drive = RobotContainer.SWERVE;
  private final OperatorControls controls = RobotContainer.operator;
  
  public ArmLocationTeleop() {
    addRequirements(arms);
  }

  @Override
  public void execute() {
    final boolean isTrimWanted = Math.abs(controls.armsMain()) > 0.0 || Math.abs(controls.armsSecondary()) > 0.0;
    final boolean isOverrideWanted = controls.isOverridePressed();

    arms.setSoftLimitSwitchEnabled(!isOverrideWanted);

    //if (!arms.isHomedPreviously() || isOverrideWanted) {
    if (isOverrideWanted) {
      //arms.setHomeLocationIfHome();
      arms.setTargetVoltage(controls.armsMain() * Constants.ARMS.TRIM.VOLTS_PER_STICK_MAIN, controls.armsSecondary() * Constants.ARMS.TRIM.VOLTS_PER_STICK_SECONDARY);
    } else if (isTrimWanted) {
      final IntakeLocation last = arms.getTargetLocation();
      final double xInches = last.xInches + Constants.ARMS.TRIM.INCHES_PER_STICK_X * RobotContainer.operator.armsSecondary() * Constants.LOOP_TIME; //TODO: fix
      final double zInches = last.zInches - Constants.ARMS.TRIM.INCHES_PER_STICK_Z * RobotContainer.operator.armsMain() * Constants.LOOP_TIME; //TODO: fix
      if(IntakeLocation.isLocationSafe(xInches, zInches)) arms.setTargetLocation(new IntakeLocation(xInches, zInches));
    } else if (drive.getAveragedSpeedMps() > Constants.SWERVE_DRIVE.TRANSPORT_SPEED_MPS) {
      arms.setTargetPositionRaw(Constants.STATE.TRANSPORT.arms);
    } else {
      arms.setTargetVoltage(0.0, 0.0);
    }
  }
}
