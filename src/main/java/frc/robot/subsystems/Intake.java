// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.team254.drivers.SparkMaxFactory;
import frc.team4646.SmartSubsystem;

public class Intake extends SmartSubsystem {
  private static class Cache {
    public double currentAmps;
    public double setpoint;
    public Cache() {
      this.currentAmps = 0.0;
      this.setpoint = 0.0;
    }
  }

  public final static String DASH_NAME = "Intake & JHook";
  private final CANSparkMax motor;
  private final Cache cache = new Cache();

  public Intake() {
    motor = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.INTAKE);
    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(80);
    motor.setOpenLoopRampRate(Constants.INTAKE.RAMP_TO_FULL_SEC);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
    motor.setIdleMode(IdleMode.kBrake);
    createDashboard();
  }

  public void setMotor(double percentOutput) {
    cache.setpoint = percentOutput;
  }

  public double getCurrentAmps() { return cache.currentAmps; }

  @Override
  public void cacheSensors() {
    cache.currentAmps = motor.getOutputCurrent();
  }

  @Override
  public void updateHardware() {
    motor.set(cache.setpoint);
  }

  public void createDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(DASH_NAME);
    tab.addDouble("Current", () -> { return getCurrentAmps(); });
    // tab.addDouble("Velocity", () -> motor.getEncoder().getVelocity());
  }
}

//yay