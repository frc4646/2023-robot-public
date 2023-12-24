// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Map;
//import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.team4646.EncoderMath;
import frc.team4646.PID;
import frc.team4646.PIDTuner;
import frc.team4646.PIDTuner.DashboardConfig;

public class Servo {
  public enum ServoMode { OPEN_LOOP, POSITION, MOTION_PROFILE }
  
  private static class Cache {
    public double positionTicks, velocityTicksPer100ms, errorNative;
    public double currentAmps;
    public boolean limitSwitchF, limitSwitchR;
    public double percentOutput;
    
    public ServoMode wantedMode;
    public double wantedSetpoint;
    public double wantedArbitraryFeedForward;

    public Cache() {
      this.positionTicks = 0.0;
      this.velocityTicksPer100ms = 0.0;
      this.errorNative = 0.0;
      this.currentAmps = 0.0;
      this.limitSwitchF = false;
      this.limitSwitchR = false;
      this.percentOutput = 0.0;
      this.wantedMode = ServoMode.OPEN_LOOP;
      this.wantedSetpoint = 0.0;
      this.wantedArbitraryFeedForward = 0.0;
    }
  }

  //public final double angleHomingDegrees;
  private final TalonFX leader;
  private final Cache cachedIO;
  //private final BooleanSupplier atHomingLocationSupplier;
  private final EncoderMath encoderMath;
  //private boolean isHomedPreviously;
  private final CANCoder encoder;

  public PIDTuner pidTuner;

  public Servo(TalonFX leader, EncoderMath encoderMath, CANCoder encoder){//, double homingPositionDegrees, BooleanSupplier funcAtHomingLocation) {
    this.encoderMath = encoderMath;
    //this.angleHomingDegrees = homingPositionDegrees;
    this.leader = leader;
    //this.atHomingLocationSupplier = funcAtHomingLocation;
    cachedIO = new Cache();

    this.encoder = encoder;

    //isHomedPreviously = false;
    setEncoder(getAngleRaw().getDegrees());
  }

  /** Changes the motor's setpoint. Call updateHardware() later to apply to motor. */
  public void setMotor(ServoMode modeWanted, double setpoint, double arbitraryFeedForward) {
    cachedIO.wantedMode = modeWanted;
    cachedIO.wantedSetpoint = setpoint;
    cachedIO.wantedArbitraryFeedForward = arbitraryFeedForward;
  }

  /** Temporarily override encoder mininum & maximum. Be very careful, only your command's logic protects the hardware when using this. */
  public void setSoftLimitSwitchEnabled(boolean limitsEnabled) { leader.overrideSoftLimitsEnable(limitsEnabled); }

  public void setEncoder(double degrees) {
    final double ticks = encoderMath.mechanismToEncoder(degrees);
    leader.setSelectedSensorPosition(ticks, 0, 0);
    cachedIO.positionTicks = ticks;
  }

  //public boolean isHomedPreviously() { return isHomedPreviously; }
  public boolean isHardwareLimitTop() { return cachedIO.limitSwitchF; }
  public boolean isHardwareLimitBottom() { return cachedIO.limitSwitchR; }
  public Rotation2d getAngleRaw() { return Rotation2d.fromDegrees(getPositionNative()); }
  public double getVelocityDegreesPerSecond() { return encoderMath.encoderToMechanism(getVelocityNative()) * 10.0; }
  public double getErrorDegrees() { return getAngleRaw().minus(Rotation2d.fromDegrees(cachedIO.wantedSetpoint)).getDegrees(); }
  public double getOutputCurrentAmps() { return cachedIO.currentAmps; }
  public double getPercentOutput() { return cachedIO.percentOutput; }
  public TalonFX getMotor() { return leader; }
  
  // Below methods return values in talon units, only useful when tuning
  public double getPositionNative() { return cachedIO.positionTicks; }
  public double getVelocityNative() { return cachedIO.velocityTicksPer100ms; }
  public double getErrorNative() { return cachedIO.errorNative; }

  /** Call once per robot code loop. Saves sensor values, handles if homing sensor is set. */
  public void cacheSensors() {
    cachedIO.positionTicks = encoder.getAbsolutePosition();
    cachedIO.velocityTicksPer100ms = encoder.getVelocity();
    cachedIO.limitSwitchR = leader.getSensorCollection().isRevLimitSwitchClosed() == 1;
    cachedIO.limitSwitchF = leader.getSensorCollection().isFwdLimitSwitchClosed() == 1;
    cachedIO.errorNative = leader.getControlMode() == ControlMode.PercentOutput ? 0.0 : leader.getClosedLoopError(0);
    cachedIO.currentAmps = leader.getStatorCurrent();
    cachedIO.percentOutput = leader.getMotorOutputPercent();

    //if(atHomingLocationSupplier.getAsBoolean()) {
    //  if (!isHomedPreviously) {
    //    leader.overrideSoftLimitsEnable(true);
    //    // TODO normally to reset location when at home sensor (outside this if statement) but not this year due to slop in system
    //    setEncoder(angleHomingDegrees);
    //  }
    //  isHomedPreviously = true;
    //}

    if(pidTuner != null) {
      pidTuner.updateMotorPIDF();
    }
  }

  /** Call once per robot code loop. <b>Actually</b> sets the motor to the previously requested setpoint. */
  public void updateHardware() {
    final double feedforward = cachedIO.wantedArbitraryFeedForward;
    switch (cachedIO.wantedMode) {
      case OPEN_LOOP:
        leader.set(TalonFXControlMode.PercentOutput, cachedIO.wantedSetpoint, DemandType.ArbitraryFeedForward, feedforward);
        break;
      case POSITION:
        leader.set(TalonFXControlMode.MotionMagic, encoderMath.mechanismToEncoder(cachedIO.wantedSetpoint), DemandType.ArbitraryFeedForward, feedforward);
        break;
      case MOTION_PROFILE:
        // TODO
        break;
      default:
        break;
    }
  }

  public ShuffleboardLayout createDashboardGrid(String tabName, String servoName, int xPosition, PID pidValues, boolean tuning) {
    ShuffleboardTab tab = Shuffleboard.getTab(tabName);
    ShuffleboardLayout grid = tab.getLayout(servoName, BuiltInLayouts.kGrid);
    
    grid.withPosition(xPosition, 0);
    grid.withSize(3, 4);
    grid.withProperties(Map.of("Number of columns", 3, "Number of rows", 5));

    int row = 0;
    grid.addDouble("Angle Raw", () -> getAngleRaw().getDegrees()).withPosition(0, row);
    if (tuning) {
      grid.addDouble("Angle Native", () -> getPositionNative()).withPosition(1, row);
    }

    row += 1;
    if (tuning) {
      grid.addDouble("Velocity", () -> getVelocityDegreesPerSecond()).withPosition(0, row);
      grid.addDouble("Velocity Native", () -> getVelocityNative()).withPosition(1, row);
    }

    row += 1;
    if (tuning) {
      grid.addDouble("Error", () -> getErrorDegrees()).withPosition(0, row);
      grid.addDouble("Error Native", () -> getErrorNative()).withPosition(1, row);
    }

    row += 1;
    if (tuning) {
      grid.addDouble("Current Amps", () -> getOutputCurrentAmps()).withPosition(0, row);
      grid.addDouble("Percent Output", () -> getPercentOutput()).withPosition(1, row);
      grid.addDouble("Wanted Angle", () -> cachedIO.wantedSetpoint).withPosition(2, row);
    }

    row += 1;
    grid.addBoolean("Top Limit", () -> isHardwareLimitTop()).withPosition(0, row);
    grid.addBoolean("Bottom Limit", () -> isHardwareLimitBottom()).withPosition(1, row);
    //grid.addBoolean("Sensor Homed", () -> isHomedPreviously()).withPosition(2, row);

    pidTuner = new PIDTuner(new DashboardConfig(tabName, servoName + " PID", xPosition + 3, 0), pidValues, getMotor());
    
    return grid;
  }

  public static CANCoderConfiguration configureCANCoder(AbsoluteSensorRange range, boolean invert, SensorInitializationStrategy initType, double offset)
  {
    CANCoderConfiguration encoderConfig;
    encoderConfig = new CANCoderConfiguration();
    encoderConfig.absoluteSensorRange = range;
    encoderConfig.sensorDirection = invert;
    encoderConfig.initializationStrategy = initType;
    encoderConfig.magnetOffsetDegrees = offset;
    encoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

    return encoderConfig;
  }
}