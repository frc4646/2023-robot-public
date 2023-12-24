package frc.robot.configuration;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.arms.ArmPosition;
import frc.robot.commands.diagnostics.ToggleGamepieceMode;
import frc.robot.commands.drive.DriveToGamePiece;
import frc.robot.commands.group.DoubleSubstation;
import frc.robot.commands.group.GamepieceIn;
import frc.robot.commands.group.GamepieceOut;
import frc.robot.commands.group.GamepiecePickup;
import frc.robot.commands.group.GamepieceScoreMiddle;
import frc.robot.commands.group.GamepieceScoreTop;
import frc.robot.commands.group.TransportMode;
import frc.robot.commands.stinger.CubeFlip;
import frc.robot.commands.wrist.WristPosition;
import frc.team4646.Util;

public class OperatorControls {
  private final CommandXboxController controller;

  public OperatorControls() {
    controller = new CommandXboxController(1);

    controller.a().whileTrue(new TransportMode());
    controller.start().toggleOnTrue(new ToggleGamepieceMode()); 
    controller.rightBumper().whileTrue(new GamepieceIn()); 
    controller.leftBumper().whileTrue(new GamepieceOut());
    controller.y().whileTrue(new DriveToGamePiece()); //Empty
    // controller.leftTrigger().whileTrue(new ()); //wrist up
    // controller.rightTrigger().whileTrue(new ()); //wrist down
    controller.povDown().whileTrue(new GamepiecePickup());
    controller.povUp().whileTrue(new GamepieceScoreTop());
    controller.povLeft().whileTrue(new DoubleSubstation()); 
    controller.povRight().whileTrue(new GamepieceScoreMiddle());
    // controller.leftStick().toggleOnTrue(new ); //Secondary ARM, what are the commands
    // controller.rightStick().toggleOnTruenew(new ); //MAIN ARM, what are the commands
    // controller.b().whileTrue(new DeployGroundIntake(0, 0)); //TODO set values
   
    //x is to EMPTY
    // LeftBumper MAIN ARM

    SmartDashboard.putData("Cube Flip", new CubeFlip(true));

    CreateDashboardArmTuning();
  }

  public void setRumble(boolean wantLeft, double percent) {
    controller.getHID().setRumble(wantLeft ? RumbleType.kLeftRumble : RumbleType.kRightRumble, percent);
  }

  public double armsSecondary() {
    return -Util.handleDeadband(controller.getRightY(), 0.2);
  }

  public double armsMain() {
    return Util.handleDeadband(controller.getLeftY(), 0.2);
  }

  public double wristOmega() {
    // return Util.handleDeadband(controller.getLeftX(), 0.2);
    double value = 0;
    if(controller.getLeftTriggerAxis() > 0) {
      value = -controller.getLeftTriggerAxis();
    }
    else if(controller.getRightTriggerAxis() > 0) {
      value = controller.getRightTriggerAxis();
    }
    return value;
  }

  public boolean isOverridePressed() {
    return controller.getHID().getBackButton();
  }

  protected void CreateDashboardArmTuning() {
    // controller.a().onTrue(new ServoPosition(Constants.ARMS.POSITION_SCORE));
    // controller.b().onTrue(new ServoPosition(Constants.ARMS.POSITION_DRIVE));
    if (Constants.TUNING.ARM_MAIN || Constants.TUNING.ARM_SECONDARY) {
      double maxMain = Constants.ARMS.MAIN.DEGREES_MAX;
      double maxSecondary = Constants.ARMS.SECONDARY.DEGREES_MAX;
      double homeMain = Constants.ARMS.MAIN.DEGREES_HOME;
      double homeSecondary = Constants.ARMS.SECONDARY.DEGREES_HOME;
      double halfMain = (Constants.ARMS.MAIN.DEGREES_MAX - Constants.ARMS.MAIN.DEGREES_MIN) / 2.0;
      double halfSecondary = (Constants.ARMS.SECONDARY.DEGREES_MAX - Constants.ARMS.SECONDARY.DEGREES_MIN) / 2.0;

      int col = 7;
      int row = 4;
      ShuffleboardTab dash = Shuffleboard.getTab(RobotContainer.ARMS.DASH_NAME);
      dash.add("HOME, HOME", new ArmPosition(homeMain, homeSecondary)).withPosition(col, row);
      col += 2;
      dash.add("HALF, HALF", new ArmPosition(halfMain, halfSecondary)).withPosition(col, row);
      col += 2;
      dash.add("MAX, HALF", new ArmPosition(maxMain, halfSecondary)).withPosition(col, row);
      col += 2;
      dash.add("HOME, HALF",new ArmPosition(homeMain, halfSecondary)).withPosition(col, row);
      col = 7;
      row += 1;
      dash.add("UP, HALF", new ArmPosition(90.0, halfSecondary)).withPosition(col, row);
      col += 2;
      dash.add("UP, FLAT", new ArmPosition(90, 0)).withPosition(col, row);
      col += 2;
      dash.add("HALF, MAX", new ArmPosition(halfMain, maxSecondary)).withPosition(col, row);
      col += 2;
      // dash.add("HALF, MAX", new ArmPosition(halfMain, maxSecondary)).withPosition(col, row);
      // col += 2;
      col = 7;
      row += 1;
      dash.add("WRIST MIN", new WristPosition(Constants.WRIST.DEGREES_HOME)).withPosition(col, row);
      col += 2;
      dash.add("WRIST HALF", new WristPosition((Constants.WRIST.DEGREES_HOME + Constants.WRIST.DEGREES_MAX) / 2)).withPosition(col, row);
      col += 2;
      dash.add("WRIST MAX", new WristPosition(Constants.WRIST.DEGREES_MAX)).withPosition(col, row);
      col += 2;
    }
  }
}
