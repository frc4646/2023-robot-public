package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.BasicTargetingData;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class DriveToGamePiece extends CommandBase{
  private final SwerveDriveSubsystem swerve = RobotContainer.SWERVE;
  private final Vision vision = RobotContainer.FRONT_VISION;
  private final double TARGET_DIFFERENCE_FACTOR = .05; 
  private final double INCHES_IN_FRONT_FACTOR = .001; 
  private final double DESIRED_INCHES_IN_FRONT = 30; 
  private final double DESIRED_Y_OFFSET = 30;
  private final double BASE_MOVE_FORWARD = 2.0;
  private double inchesInFront;
  private BasicTargetingData target;

  public void execute()
  {
    Translation2d move = new Translation2d(BASE_MOVE_FORWARD, 0.0); //DriveTeleopHelper.getMove();
    double rotate = 0;
    Translation2d centerOfRotation = new Translation2d(Units.inchesToMeters(40.0), 0.0);

    if(vision.foundGamePiece()) {
      target = vision.getGamePiecePosition();
      if(target != null)
      {        
        inchesInFront = convertHeightToInces(target.horizontal);  // camera rotated 90
        // final double gamepieceWidthScaling = 1.0;
        final double aspectRatio = target.vertical/target.horizontal;
        final double gamepieceWidthScaling = scaleToRange(aspectRatio, 1.0, 1.65, 0.25, 1.0);  // camera rotated 90

        rotate *= gamepieceWidthScaling;
        centerOfRotation = new Translation2d(Units.inchesToMeters(inchesInFront), 0.0);
        double y = -target.yOffset;
        if(target.yOffset < 0.1)
        {
          y = 0;

        }
        // if(inchesInFront < DESIRED_INCHES_IN_FRONT)
        final Translation2d targetDifference = new Translation2d(INCHES_IN_FRONT_FACTOR * inchesInFront, TARGET_DIFFERENCE_FACTOR * y);
        move = move.plus(targetDifference);
        SmartDashboard.putNumber("target difference left/right", targetDifference.getY());
        SmartDashboard.putNumber("distance to game piece from drivetogamepiece", inchesInFront);
      }

    }      
    SmartDashboard.putNumber("commanded move x ", move.getX());
    SmartDashboard.putNumber("commanded move y ", move.getY());
    SmartDashboard.putNumber("Commanded rotate", rotate);

    swerve.drive(move, rotate, false, false, centerOfRotation);
  }
  
  public boolean isFinished(){
    return true; //TODO Finish this function
  //return RobotContainer.DIAGNOSTICS.gamePieceInFront;
    // if(target != null) {
    //   return inchesInFront < DESIRED_INCHES_IN_FRONT && -target.yOffset < DESIRED_Y_OFFSET;
    // } else {
    //   return true;
    // }
  }

  private double convertHeightToInces(double height) {
    return (0.0003 * Math.pow(height, 2.0) ) - (0.3122 * height) + 109.36; // just a dummy equation from excel, there's fancy math to be done here
  }

  protected double scaleToRange(double in, double in_min, double in_max, double out_min, double out_max) {
    return (in - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
}
