// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.configuration.IDriverControls;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Robotstate;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class DriveTeleop extends CommandBase {
  private final SwerveDriveSubsystem swerve = RobotContainer.SWERVE;
  private final Robotstate state = RobotContainer.STATE;
  private final ArmSubsystem arms = RobotContainer.ARMS;
  private final Vision vision = RobotContainer.FRONT_VISION;
  private final IDriverControls driver = RobotContainer.driver;
  private boolean fieldRelative = true;
  private final double targetDifferenceFactor = .05;
  // private final double ALIGNED_THRESHOLD = .1; 
  ShuffleboardTab dash;
  double yVisionOffset, xVisionOffset;

  private final boolean OPEN_LOOP_SPEED = true;
    
  public DriveTeleop() {
    addRequirements(swerve);
    dash = Shuffleboard.getTab(vision.DASH_NAME);
    // dash.addNumber("target adjustmet",() -> targetDifferenceFactor * -yVisionOffset );
  }

  @Override
  public void initialize() {
    swerve.setBrakeMode(false);
  }

  @Override
  public void execute() {
    Translation2d move = DriveTeleopHelper.getMove();
    double rotationRate = DriveTeleopHelper.getRotationRate();

    if (driver.getFieldRelativeToggle()) {
      fieldRelative = !fieldRelative;
    }

    // if arm is raised, slow down
    if (arms.getAngleSecondaryRaw().getDegrees() > Constants.ARMS.ARM_SECONDARY_RAISED) {
      move = move.times(0.5);
      rotationRate = rotationRate * 0.5;
    }

    // if scoring a cone, automatically align with the node
    if(driver.getSpeedSlow() &&  vision.foundVisionTape() && state.isConeMode() && RobotContainer.CANIFIER.isConeDetected()) {
      yVisionOffset = vision.getVisionTapePosition().yOffset;
      xVisionOffset = vision.getVisionTapePosition().xOffset;
      Translation2d targetDifference = new Translation2d(0.0, targetDifferenceFactor * yVisionOffset);
      // SmartDashboard.putNumber("targetDifference", targetDifference.getY());
      move = move.plus(targetDifference);      
      // rotate = 0 // point towards goal
    }
    // else if (driver.getSpeedSlow() && vision.foundGamePiece()) {
    //   final double errorRotate = vision.getGamePiecePosition().yOffset;
    //   rotate = swerve.getGyroAngle().getDegrees() + errorRotate;
    //   fieldRelative = true;
    // }

    swerve.drive(move, rotationRate, fieldRelative, OPEN_LOOP_SPEED);
  }
}
