// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Robotstate;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.VisionPipeline;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class VisionFront extends CommandBase {
  private final Vision subsystem = RobotContainer.FRONT_VISION;
  private final Robotstate state = RobotContainer.STATE;
  private final SwerveDriveSubsystem drive = RobotContainer.SWERVE;

  public VisionFront() {
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    subsystem.setPipeline(VisionPipeline.APRILTAG);
  }
  
  @Override
  public void execute() {
    subsystem.setLEDs(RobotContainer.CANIFIER.isConeDetected());
  }
}
