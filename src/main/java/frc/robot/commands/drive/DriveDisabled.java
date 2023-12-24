// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class DriveDisabled extends CommandBase {
  private final SwerveDriveSubsystem drive = RobotContainer.SWERVE;
  private double timeStarted = Double.MAX_VALUE;

  @Override
  public void initialize() {
    timeStarted = Timer.getFPGATimestamp();
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - timeStarted > Constants.SWERVE_DRIVE.TIMEOUT_DISABLED_COAST;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Setting coast!");
    drive.setBrakeMode(false);  // Delay so inertia cancels before allowing coast
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
