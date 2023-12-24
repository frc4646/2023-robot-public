// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.BasicTargetingData;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class DriveTeleopAroundCone extends CommandBase {
  private final double TARGET_DIFFERENCE_FACTOR = .05; 
  
  private final SwerveDriveSubsystem swerve = RobotContainer.SWERVE;
  private final Vision vision = RobotContainer.FRONT_VISION;

  public DriveTeleopAroundCone() {
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    
    Translation2d move = DriveTeleopHelper.getMove();
    double rotate = -DriveTeleopHelper.getRotationRate() * 0.5;
    Translation2d centerOfRotation = new Translation2d(Units.inchesToMeters(40.0), 0.0);

    if(vision.foundGamePiece()) {
      final BasicTargetingData target = vision.getGamePiecePosition();
      final Translation2d targetYDifference = new Translation2d(0.0, TARGET_DIFFERENCE_FACTOR * -target.yOffset);
      final double inchesInFront = convertHeightToInces(target.horizontal);  // camera rotated 90
      // final double gamepieceWidthScaling = 1.0;
      final double aspectRatio = target.vertical/target.horizontal;
      final double gamepieceWidthScaling = scaleToRange(aspectRatio, 1.0, 1.65, 0.25, 1.0);  // camera rotated 90

      move = move.plus(targetYDifference);
      rotate *= gamepieceWidthScaling;
      centerOfRotation = new Translation2d(Units.inchesToMeters(inchesInFront), 0.0);

      SmartDashboard.putNumber("target difference", targetYDifference.getY());
      SmartDashboard.putNumber("inchesInFront", inchesInFront);
    }
    swerve.drive(move, rotate, false, false, centerOfRotation);
  }

  private double convertHeightToInces(double height) {
    return (0.0003 * Math.pow(height, 2.0) ) - (0.3122 * height) + 109.36; // just a dummy equation from excel, there's fancy math to be done here
  }

  protected double scaleToRange(double in, double in_min, double in_max, double out_min, double out_max) {
    return (in - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
}
