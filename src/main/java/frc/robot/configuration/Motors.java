// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.configuration;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.Pair;
import frc.robot.Constants;

public class Motors {
  public static Pair<TalonFX,CANCoder> createMain() {
    return new TalonFXSettings(Constants.ARMS.MAIN.TALONFX, Constants.ARMS.MAIN.CANCODER)
    .configureOpenLoop(0.4)
    // .configureCurrentLimits(30.0, 35.0, 0.04)
    .configureRemoteEncoder(false)
    .configureMotionMagic(Constants.ARMS.MAIN.MOTION_MAGIC)
    .configureHardwareLimits(Constants.ARMS.MAIN.LIMITS_HARDWARE)
    .configureSoftwareLimits(Constants.ARMS.MAIN.LIMITS_SOFTWARE)
    .get();
  }

  public static Pair<TalonFX,CANCoder> createSecondary() {
    final TalonFX motor = TalonFXSettings.createMotor(Constants.ARMS.SECONDARY.TALONFX);
    final CANCoder encoder = TalonFXSettings.createEncoder(Constants.ARMS.SECONDARY.CANCODER);
    
    TalonFXSettings.configureOpenLoop(motor, 0.8);
    //TalonFXBuilder.configureCurrentLimits(motor, 30.0, 35.0, 0.04);
    TalonFXSettings.configureRemoteEncoder(motor, encoder, true);
    TalonFXSettings.configureMotionMagic(motor, Constants.ARMS.SECONDARY.MOTION_MAGIC);
    TalonFXSettings.configureHardwareLimits(motor, Constants.ARMS.SECONDARY.LIMITS_HARDWARE);
    TalonFXSettings.configureSoftwareLimits(motor, Constants.ARMS.SECONDARY.LIMITS_SOFTWARE);
    
    return new Pair<TalonFX,CANCoder>(motor, encoder);
  }

  public static Pair<TalonFX,CANCoder> createWrist() {
    final TalonFX motor = TalonFXSettings.createMotor(Constants.WRIST.TALONFX);
    final CANCoder encoder = TalonFXSettings.createEncoder(Constants.WRIST.CANCODER);
    
    TalonFXSettings.configureOpenLoop(motor, 0.4);
    //TalonFXBuilder.configureCurrentLimits(motor, 30.0, 35.0, 0.04);
    TalonFXSettings.configureRemoteEncoder(motor, encoder, false);
    TalonFXSettings.configureMotionMagic(motor, Constants.WRIST.MOTION_MAGIC);
    TalonFXSettings.configureHardwareLimits(motor, Constants.WRIST.LIMITS_HARDWARE);
    TalonFXSettings.configureSoftwareLimits(motor, Constants.WRIST.LIMITS_SOFTWARE);

    return new Pair<TalonFX,CANCoder>(motor, encoder);
  }
}
