// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.configuration.Motors;
import frc.robot.util.ArmAngles;
import frc.robot.util.IntakeLocation;
import frc.robot.util.Servo;
import frc.robot.util.Servo.ServoMode;
import frc.team4646.SmartSubsystem;

public class ArmSubsystem extends SmartSubsystem {
  public final String DASH_NAME = "Arms";

  private final Servo jointMain;
  private final Servo jointSecondary;
  private IntakeLocation targetLocation;
  private ArmAngles targetPositionRaw;

  public ArmSubsystem() {
    Pair<TalonFX,CANCoder> main = Motors.createMain();
    jointMain = new Servo(main.getFirst(), Constants.ARMS.MAIN.ENCODER_MATH, main.getSecond());
    
    Pair<TalonFX,CANCoder> secondary  = Motors.createSecondary();
    jointSecondary = new Servo(secondary.getFirst(), Constants.ARMS.SECONDARY.ENCODER_MATH, secondary.getSecond());

    targetLocation = new IntakeLocation();
    targetPositionRaw = new ArmAngles();

    createDashboard();
  }

  /** Control arms by updating motor setpoints using wanted <b>percent voltage</b> */
  public void setTargetVoltage(double percentMain, double percentSecondary) {

    var currentAngles = getAnglesRaw();
    if(percentMain > 0.0 && isMainArmSaturatedMax(currentAngles)) {
      percentMain = 0.0;
    }
    if(percentMain < 0.0 && isMainArmSaturatedMin(currentAngles)) {
      percentMain = 0.0;
    }
    if(percentSecondary > 0.0 && isSecondaryArmSaturatedMax(currentAngles)) {
      percentSecondary = 0.0;
    }
    if(percentSecondary < 0.0 && isSecondaryArmSaturatedMin(currentAngles)) {
      percentSecondary = 0.0;
    }

    if(percentMain > 0.0 || percentSecondary > 0.0)
    {
      //Moving an arm out in open loop, make sure we are still legal
      if(IntakeLocation.isLocationLegal(getAnglesFromGround(), RobotContainer.WRIST.getAngleFromGround().getDegrees()))
      {
        targetPositionRaw = getAnglesRaw();
        targetLocation = getLocationInches();
        setMotors(ServoMode.OPEN_LOOP, percentMain, percentSecondary);
      }
    }
    else
    {
      targetPositionRaw = getAnglesRaw();
      targetLocation = getLocationInches();
      setMotors(ServoMode.OPEN_LOOP, percentMain, percentSecondary);
    }

  }

  /** Control arms by updating motor setpoints using wanted <b>position control</b> */
  public void setTargetPositionRaw(ArmAngles wanted) {
    // if(IntakeLocation.isLocationLegal(boundToEnvelope(wanted), RobotContainer.WRIST.getAngle().getDegrees()))
    {
      targetPositionRaw = boundToEnvelope(wanted);
      ArmAngles targetFromGround = new ArmAngles(Rotation2d.fromDegrees(targetPositionRaw.mainDegrees), 
                                                 calcAngleSecondaryFromGround(targetPositionRaw.mainDegrees, targetPositionRaw.secondaryDegrees));
      targetLocation = ArmAngles.toLocation(targetFromGround);
      setMotors(ServoMode.POSITION, targetPositionRaw.mainDegrees, targetPositionRaw.secondaryDegrees);
    }
  }

  /** Control arms by updating motor setpoints using wanted <b>location of the intake</b> */
  public void setTargetLocation(IntakeLocation wanted)	{
    final ArmAngles position = boundToEnvelope(IntakeLocation.toPosition(wanted));
    
    if (!isEitherArmSaturated(position) && IntakeLocation.isLocationLegal(position, RobotContainer.WRIST.getAngleFromGround().getDegrees())) 
    {
      targetLocation = wanted;
      targetPositionRaw = position;
      setMotors(ServoMode.POSITION, position.mainDegrees, position.secondaryDegrees);
    }
  }

  /** Temporarily override encoder mininum & maximum. Be very careful, only your command's logic protects the hardware when using this. */
  public void setSoftLimitSwitchEnabled(boolean limitsEnabled) { 
    jointMain.setSoftLimitSwitchEnabled(limitsEnabled);
    jointSecondary.setSoftLimitSwitchEnabled(limitsEnabled);
  }

  //public void setHomeLocationIfHome() {
  //  if (isHomedMain() && isHomedSecondary()) {
  //    jointMain.setEncoder(Constants.ARMS.MAIN.LOCATIONS.HOME_RAW_DEGREES);
  //    jointSecondary.setEncoder(Constants.ARMS.SECONDARY.LOCATIONS.HOME_RAW_DEGREES);
  //  }
  //}

  //public boolean isHomedPreviously() { return jointMain.isHomedPreviously() && jointSecondary.isHomedPreviously(); }
  public boolean isHomedMain() { return jointMain.isHardwareLimitTop(); }
  public boolean isHomedSecondary() { return jointSecondary.isHardwareLimitBottom(); }
  
  public boolean isOnTargetPositionMain() { return Math.abs(jointMain.getErrorDegrees()) < Constants.ARMS.MAIN.ON_TARGET_DEGREES; }
  public boolean isOnTargetPositionSecondary() { return Math.abs(jointSecondary.getErrorDegrees()) < Constants.ARMS.SECONDARY.ON_TARGET_DEGREES; }
  public boolean isOnTargetVelocityMain() { return Math.abs(jointMain.getVelocityDegreesPerSecond()) < Constants.ARMS.MAIN.ON_TARGET_DEGREES_PER_SECOND; }
  public boolean isOnTargetVelocitySecondary() { return Math.abs(jointSecondary.getVelocityDegreesPerSecond()) < Constants.ARMS.SECONDARY.ON_TARGET_DEGREES_PER_SECOND; }

  public ArmAngles getAnglesRaw() { return new ArmAngles(getAngleMainRaw(), getAngleSecondaryRaw()); }
  public ArmAngles getAnglesFromGround() { return new ArmAngles(getAngleMainFromGround(), getAngleSecondaryFromGround()); }
  public IntakeLocation getLocationInches() {return ArmAngles.toLocation(getAnglesFromGround());}

  public Rotation2d getAngleMainFromGround() { return jointMain.getAngleRaw(); }
  public Rotation2d getAngleMainRaw() { return jointMain.getAngleRaw(); }

  public Rotation2d getAngleSecondaryFromGround() { return calcAngleSecondaryFromGround(jointMain.getAngleRaw(), jointSecondary.getAngleRaw()); }
  public Rotation2d getAngleSecondaryRaw() { return jointSecondary.getAngleRaw();}
  public double getVelocityDegreesPerSecondMain() { return jointMain.getVelocityDegreesPerSecond(); }
  public double getVelocityDegreesPerSecondSecondary() { return jointSecondary.getVelocityDegreesPerSecond(); }
  
  public IntakeLocation getTargetLocation() { return targetLocation; }

  /** Convert raw angles in degrees to angle of secondary from the ground */
  public Rotation2d calcAngleSecondaryFromGround(double mainRawDegrees, double secondaryRawDegrees) {
    return calcAngleSecondaryFromGround(Rotation2d.fromDegrees(mainRawDegrees), Rotation2d.fromDegrees(secondaryRawDegrees));
  }
  /** Convert raw angles to angle of secondary from the ground */
  public Rotation2d calcAngleSecondaryFromGround(Rotation2d mainRaw, Rotation2d secondaryRaw) {
    return (Rotation2d.fromDegrees(-179.9)
            .plus(mainRaw)
            .plus(secondaryRaw)
            .minus(Rotation2d.fromDegrees(.1)));
  }

  private void setMotors(ServoMode modeWanted, double mainWantedRaw, double secondaryWanted) {
    ServoMode mainMode = modeWanted;
    double main = mainWantedRaw;
    double mainAFF = getArbitraryFeedForwardMain();

    ServoMode secondaryMode = modeWanted;
    double secondary = secondaryWanted;//modeWanted == ServoMode.POSITION ? 180.0 + secondaryWanted - getAngleMainRaw().getDegrees() : secondaryWanted;
    double secondaryAFF = getArbitraryFeedForwardSecondary();

    // Protect gearbox from main arm
    if (jointSecondary.isHardwareLimitBottom() && (modeWanted == ServoMode.OPEN_LOOP && mainWantedRaw > 0.0)) {
      mainMode = ServoMode.OPEN_LOOP;
      main = 0.0;
      mainAFF = 0.0;
    }
    
    jointMain.setMotor(mainMode, main, mainAFF);
    jointSecondary.setMotor(secondaryMode, secondary, secondaryAFF);
  }

  public double getArbitraryFeedForwardMain() {
    return getAngleMainFromGround().getCos() * Constants.ARMS.MAIN.GRAVITY_FF_PERCENT_VOLTAGE +
           getAngleSecondaryFromGround().getCos() * Constants.ARMS.SECONDARY.GRAVITY_OFFSET_FF * .5;
    //        RobotContainer.WRIST.getAngle().getCos() * Constants.WRIST.GRAVITY_OFFSET_FF;
  }

  public double getArbitraryFeedForwardSecondary() {
    return getAngleSecondaryFromGround().getCos() * Constants.ARMS.SECONDARY.GRAVITY_OFFSET_FF +
           RobotContainer.WRIST.getAngleFromGround().getCos() * Constants.WRIST.GRAVITY_OFFSET_FF;
  }

  private boolean isEitherArmSaturated(ArmAngles wanted) {
    return isMainArmSaturated(wanted) || isSecondaryArmSaturated(wanted);
  }
  private boolean isMainArmSaturated(ArmAngles wanted) {
    return isMainArmSaturatedMin(wanted) || isMainArmSaturatedMax(wanted);
  }
  private boolean isSecondaryArmSaturated(ArmAngles wanted) {
    return isSecondaryArmSaturatedMin(wanted) || isSecondaryArmSaturatedMax(wanted);
  }
  private boolean isMainArmSaturatedMin(ArmAngles wanted) {
    return (wanted.mainDegrees < Constants.ARMS.ENVELOPE.getMinDegreesMain());
  }
  private boolean isMainArmSaturatedMax(ArmAngles wanted) {
    return (wanted.mainDegrees > Constants.ARMS.ENVELOPE.getMaxDegreesMain());
  }
  private boolean isSecondaryArmSaturatedMin(ArmAngles wanted) {
    return (wanted.secondaryDegrees < Constants.ARMS.ENVELOPE.getEnvelopMinDegreesSecondary(wanted.mainDegrees));
  }
  private boolean isSecondaryArmSaturatedMax(ArmAngles wanted) {
    return (wanted.secondaryDegrees > Constants.ARMS.ENVELOPE.getEnvelopMaxDegreesSecondary(wanted.mainDegrees));
  }

  private ArmAngles boundToEnvelope(ArmAngles wanted) {
    final double minMain = Constants.ARMS.ENVELOPE.getMinDegreesMain();
    final double maxMain = Constants.ARMS.ENVELOPE.getMaxDegreesMain();
    final double minSecondary = Constants.ARMS.ENVELOPE.getEnvelopMinDegreesSecondary(getAngleMainRaw().getDegrees());
    final double maxSecondary = Constants.ARMS.ENVELOPE.getEnvelopMaxDegreesSecondary(getAngleMainRaw().getDegrees());
    double setpointMain = wanted.mainDegrees;
    double setpointSecondary = wanted.secondaryDegrees;

    if (setpointMain < minMain) {
      setpointMain = minMain;
    }
    if (setpointMain > maxMain) {
      setpointMain = maxMain;
    }
    if (setpointSecondary < minSecondary) {
      setpointSecondary = minSecondary;
    }
    if (setpointSecondary > maxSecondary) {
      setpointSecondary = maxSecondary;
    }
    return new ArmAngles(setpointMain, setpointSecondary);
  }

  @Override
  public void cacheSensors() {
    jointMain.cacheSensors();
    jointSecondary.cacheSensors();
  }
  
  @Override
  public void updateHardware() {
    jointMain.updateHardware();
    jointSecondary.updateHardware();
  }

  public void createDashboard() {
    jointMain.createDashboardGrid(DASH_NAME, "Main", 0, Constants.ARMS.MAIN.MOTION_MAGIC.pid, Constants.TUNING.ARM_MAIN);
    jointSecondary.createDashboardGrid(DASH_NAME, "Secondary", 4, Constants.ARMS.SECONDARY.MOTION_MAGIC.pid, Constants.TUNING.ARM_SECONDARY);
    // Constants.ARMS.ENVELOPE.createDashboardGrid(DASH_NAME, "Envelop", 9, () -> getAngleDegreesMain());
    Shuffleboard.getTab(DASH_NAME).add(this).withPosition(0, 4);
    Shuffleboard.getTab(DASH_NAME).addBoolean("On Target Main", () -> isOnTargetPositionMain()).withPosition(2, 4);
    Shuffleboard.getTab(DASH_NAME).addBoolean("On Target Secondary", () -> isOnTargetPositionSecondary()).withPosition(4, 4);
    Shuffleboard.getTab(DASH_NAME).addDouble("Main From Ground", ()->getAngleMainFromGround().getDegrees()).withPosition(3, 3);
    Shuffleboard.getTab(DASH_NAME).addDouble("Secondary From Ground", ()->getAngleSecondaryFromGround().getDegrees()).withPosition(7, 3);
    // Shuffleboard.getTab(DASH_NAME).addDouble("Wrist from Ground", ()->RobotContainer.WRIST.getAngle().getDegrees());
    // Shuffleboard.getTab(DASH_NAME).addDouble("X Position", ()->getLocationInches().xInches);
    // Shuffleboard.getTab(DASH_NAME).addDouble("Z Position", ()->getLocationInches().zInches);
  }
}
