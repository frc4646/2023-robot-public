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
import frc.robot.util.Servo;
import frc.robot.util.Servo.ServoMode;
import frc.team4646.SmartSubsystem;

public class WristSubsystem extends SmartSubsystem {
  private final ArmSubsystem arms;
  private final Servo motor;
  private double targetDegrees;
  // private double rawTargetDegrees;

  public WristSubsystem() {
    Pair<TalonFX,CANCoder> pair = Motors.createWrist();
    motor = new Servo(pair.getFirst(), Constants.WRIST.ENCODER_MATH, pair.getSecond());

    arms = RobotContainer.ARMS;
    targetDegrees = 0.0;

    motor.createDashboardGrid(RobotContainer.ARMS.DASH_NAME, "Wrist", 8, Constants.WRIST.MOTION_MAGIC.pid, Constants.TUNING.WRIST);
    Shuffleboard.getTab(RobotContainer.ARMS.DASH_NAME).addBoolean("On Target Wrist", () -> isOnTarget()).withPosition(4, 6);
  }

  public void setVoltage(double percentVoltage) {
    // if(IntakeLocation.isLocationLegal(RobotContainer.ARMS.getAnglesRaw(), getAngle().getDegrees()))
    {
      targetDegrees = getAngleRaw().getDegrees();
      motor.setMotor(ServoMode.OPEN_LOOP, percentVoltage, getArbitraryFeedForward());
    }
  }

  public void setPosition(double degrees) {    
    // if(IntakeLocation.isLocationLegal(RobotContainer.ARMS.getAnglesRaw(), getAngle().getDegrees()))
    {
      targetDegrees = degrees;
      // rawTargetDegrees = targetDegrees + arms.getAngleSecondaryRaw().getDegrees() - 180.0;
      motor.setMotor(ServoMode.POSITION, targetDegrees, getArbitraryFeedForward());
    }
  }

  public void setSoftLimitSwitchEnabled(boolean enabled) {
    motor.setSoftLimitSwitchEnabled(enabled);
  }

  //public void setHomeLocationIfHome() {
  //  if (isHome()) {
  //    motor.setEncoder(Constants.WRIST.LOCATIONS.HOME_RAW_DEGREES);
  //  }
  //}

  public boolean isHome() { return motor.isHardwareLimitTop(); }
  //public boolean isHomedPreviously() { return motor.isHomedPreviously(); }
  public boolean isOnTarget() { return Math.abs(motor.getErrorDegrees()) < Constants.WRIST.ON_TARGET_DEGREES; }
  public Rotation2d getAngleFromGround() {
    return Rotation2d.fromDegrees(180.0)
           .minus(motor.getAngleRaw())
           .minus(arms.getAngleSecondaryFromGround());
  }
  public Rotation2d getAngleRaw() {return motor.getAngleRaw(); }
  public double getTarget() { return targetDegrees; }

  private double getArbitraryFeedForward() {
    return getAngleFromGround().getCos() * 
          (Constants.WRIST.GRAVITY_OFFSET_FF + (RobotContainer.CANIFIER.isConeDetected() ? .002 : 0.0));
  }

  @Override
  public void cacheSensors() {
    motor.cacheSensors();
  }

  @Override
  public void updateHardware() {
    motor.updateHardware();
  }
}
