package frc.team4646;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;
import frc.team254.drivers.TalonFXFactory;
import frc.team254.util.Util;

/** Abstract base class for a subsystem with a single sensored servo-mechanism */
public abstract class ServoMotorSubsystem extends SmartSubsystem {
  private static final int kMotionProfileSlot = 0, kPositionPIDSlot = 1;

  /** Recommend initializing in a static block! */
  public static class TalonFXConstants {
    public String canBus = "rio";
    public int id = -1;
    public boolean invert_motor = false;
    public boolean invert_sensor_phase = false;
    public double ticksPerMotorRotation = 2048.0;
  }

  /** Recommend initializing in a static block! */
  public static class ServoMotorSubsystemConstants {
    public TalonFXConstants kLeaderConstants = new TalonFXConstants();
    public TalonFXConstants[] kFollowerConstants = new TalonFXConstants[0];

    public double kHomePosition = 0.0; // Units
    public double kTicksPerUnitDistance = 1.0;
    public PID kMotionMagicPID = new PID();
    public double kMaxIntegralAccumulator = 0.0;
    public double kIZone = 0.0; // Ticks
    public double kMotionMagicDeadband = 0.0; // Ticks

    public PID kPositionPID = new PID();
    public double kPositionMaxIntegralAccumulator = 0.0;
    public double kPositionIZone = 0.0; // Ticks
    public double kPositionDeadband = 0.0; // Ticks

    public double kCruiseVelocity = 0.0; // Ticks / 100ms
    public double kAcceleration = 0.0; // Ticks / 100ms / s
    public double kRampRate = 0.0; // s
    public double kMaxVoltage = 12.0;

    public SupplyCurrentLimitConfiguration kSupplyCurrentLimit = new SupplyCurrentLimitConfiguration(false, 20.0, 60.0, 0.2);
    public StatorCurrentLimitConfiguration kStatorCurrentLimit = new StatorCurrentLimitConfiguration(false, 20.0, 60.0, 0.2);

    public double kMaxUnitsLimit = Double.POSITIVE_INFINITY;
    public double kMinUnitsLimit = Double.NEGATIVE_INFINITY;

    public int kStatusFrame2UpdateRate = 10;
    public int kStatusFrame10UpdateRate = 10;
    public int kStatusFrame8UpdateRate = 1000;
    public boolean kRecoverPositionOnReset = false;
  }

  protected final ServoMotorSubsystemConstants mConstants;
  protected final TalonFX mLeader;
  protected final TalonFX[] mFollowers;

  protected final double mForwardSoftLimitTicks, mReverseSoftLimitTicks;

  protected ServoMotorSubsystem(final ServoMotorSubsystemConstants constants) {
    mConstants = constants;
    mLeader = TalonFXFactory.createDefaultTalon(mConstants.kLeaderConstants.id, mConstants.kLeaderConstants.canBus);
    mFollowers = new TalonFX[mConstants.kFollowerConstants.length];

    TalonUtil.checkError(mLeader.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT), getName() + ": Could not detect encoder: ");

    mForwardSoftLimitTicks = (mConstants.kMaxUnitsLimit - mConstants.kHomePosition) * mConstants.kTicksPerUnitDistance;
    mReverseSoftLimitTicks = (mConstants.kMinUnitsLimit - mConstants.kHomePosition) * mConstants.kTicksPerUnitDistance;
    TalonUtil.checkError(mLeader.configForwardSoftLimitThreshold(mForwardSoftLimitTicks, Constants.CAN_TIMEOUT), getName() + ": Could not set forward soft limit: ");
    TalonUtil.checkError(mLeader.configForwardSoftLimitEnable(true, Constants.CAN_TIMEOUT), getName() + ": Could not enable forward soft limit: ");
    TalonUtil.checkError(mLeader.configReverseSoftLimitThreshold(mReverseSoftLimitTicks, Constants.CAN_TIMEOUT), getName() + ": Could not set reverse soft limit: ");
    TalonUtil.checkError(mLeader.configReverseSoftLimitEnable(true, Constants.CAN_TIMEOUT), getName() + ": Could not enable reverse soft limit: ");

    TalonUtil.checkError(mLeader.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT), getName() + ": Could not set voltage compensation saturation: ");

    TalonFXFactory.setPID(mLeader, mConstants.kMotionMagicPID, kMotionProfileSlot);
    TalonUtil.checkError(mLeader.configMaxIntegralAccumulator(kMotionProfileSlot, mConstants.kMaxIntegralAccumulator, Constants.CAN_TIMEOUT), getName() + ": Could not set max integral: ");
    TalonUtil.checkError(mLeader.config_IntegralZone(kMotionProfileSlot, mConstants.kIZone, Constants.CAN_TIMEOUT), getName() + ": Could not set i zone: ");
    TalonUtil.checkError(mLeader.configAllowableClosedloopError(kMotionProfileSlot, mConstants.kMotionMagicDeadband, Constants.CAN_TIMEOUT), getName() + ": Could not set deadband: ");
    TalonUtil.checkError(mLeader.configMotionCruiseVelocity(mConstants.kCruiseVelocity, Constants.CAN_TIMEOUT), getName() + ": Could not set cruise velocity: ");
    TalonUtil.checkError(mLeader.configMotionAcceleration(mConstants.kAcceleration, Constants.CAN_TIMEOUT), getName() + ": Could not set acceleration: ");

    TalonFXFactory.setPID(mLeader, mConstants.kPositionPID, kPositionPIDSlot);
    TalonUtil.checkError(mLeader.configMaxIntegralAccumulator(kPositionPIDSlot, mConstants.kPositionMaxIntegralAccumulator, Constants.CAN_TIMEOUT), getName() + ": Could not set max integral: ");
    TalonUtil.checkError(mLeader.config_IntegralZone(kPositionPIDSlot, mConstants.kPositionIZone, Constants.CAN_TIMEOUT), getName() + ": Could not set i zone: ");
    TalonUtil.checkError(mLeader.configAllowableClosedloopError(kPositionPIDSlot, mConstants.kPositionDeadband, Constants.CAN_TIMEOUT), getName() + ": Could not set deadband: ");

    TalonUtil.checkError(mLeader.configOpenloopRamp(mConstants.kRampRate, Constants.CAN_TIMEOUT), getName() + ": Could not set voltage ramp rate: ");
    TalonUtil.checkError(mLeader.configClosedloopRamp(mConstants.kRampRate, Constants.CAN_TIMEOUT), getName() + ": Could not set closed loop ramp rate: ");

    TalonUtil.checkError(mLeader.configSupplyCurrentLimit(mConstants.kSupplyCurrentLimit), getName() + ": Could not set supply current limit.");
    TalonUtil.checkError(mLeader.configStatorCurrentLimit(mConstants.kStatorCurrentLimit), getName() + ": Could not set stator current limit.");

    mLeader.configVoltageMeasurementFilter(8);
    TalonUtil.checkError(mLeader.configVoltageCompSaturation(mConstants.kMaxVoltage, Constants.CAN_TIMEOUT), getName() + ": Could not set voltage comp saturation.");
    mLeader.enableVoltageCompensation(true);

    mLeader.setInverted(mConstants.kLeaderConstants.invert_motor);
    mLeader.setSensorPhase(mConstants.kLeaderConstants.invert_sensor_phase);
    mLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, mConstants.kStatusFrame2UpdateRate, Constants.CAN_TIMEOUT);
    mLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, mConstants.kStatusFrame10UpdateRate, Constants.CAN_TIMEOUT);
    mLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, mConstants.kStatusFrame8UpdateRate, Constants.CAN_TIMEOUT);
    mLeader.selectProfileSlot(kMotionProfileSlot, 0);  // Start with kMotionProfileSlot

    for (int i = 0; i < mFollowers.length; ++i) {
      mFollowers[i] = TalonFXFactory.createPermanentFollowerTalon(mConstants.kFollowerConstants[i].id, mConstants.kFollowerConstants[i].canBus, mLeader);
      mFollowers[i].setInverted(mConstants.kFollowerConstants[i].invert_motor);  // TODO change to new follow/oppose leader invert type
    }

    setBrakeMode(!isBrakeMode);
    setOpenLoop(0.0);
    mLeader.set(TalonFXControlMode.PercentOutput, 0.0);  // Send a neutral command

    Shuffleboard.getTab(getName()).addDouble("Position", () -> mPeriodicIO.position_units);
  }

  public static class PeriodicIO {
    // INPUTS
    public double position_ticks;
    public double position_units;
    public double velocity_ticks_per_100ms;
    public double active_trajectory_position; // ticks
    public double active_trajectory_velocity; // ticks/100ms
    public double active_trajectory_acceleration; // ticks/100ms/s
    public double error_ticks;
    public double encoder_wraps;
    public double absolute_pulse_offset = 0;
    public double absolute_pulse_position_modded;
    public boolean reset_occured;
    // OUTPUTS
    public double demand;
    public double feedforward;
  }
  protected enum ControlState {
    OPEN_LOOP, MOTION_MAGIC, POSITION_PID
  }

  protected PeriodicIO mPeriodicIO = new PeriodicIO();
  protected ControlState mControlState = ControlState.OPEN_LOOP;
  protected boolean mHasBeenZeroed = false;
  protected StickyFaults mFaults = new StickyFaults();
  protected boolean isBrakeMode = false;

  @Override
  public void cacheSensors() {
    if (mLeader.hasResetOccurred()) {
      DriverStation.reportError(getName() + ": Talon Reset! ", false);
      mPeriodicIO.reset_occured = true;
      return;
    } 
    mPeriodicIO.reset_occured = false;

    mLeader.getStickyFaults(mFaults);
    if (mFaults.hasAnyFault()) {
      DriverStation.reportError(getName() + ": Talon Fault! " + mFaults.toString(), false);
      mLeader.clearStickyFaults(0);
    }
    if (mLeader.getControlMode() == ControlMode.MotionMagic) {
      mPeriodicIO.active_trajectory_position = mLeader.getActiveTrajectoryPosition();

      if (mPeriodicIO.active_trajectory_position < mReverseSoftLimitTicks) {
        DriverStation.reportError(getName() + ": Active trajectory past reverse soft limit!", false);
      } else if (mPeriodicIO.active_trajectory_position > mForwardSoftLimitTicks) {
        DriverStation.reportError(getName() + ": Active trajectory past forward soft limit!", false);
      }
      final double newVel = mLeader.getActiveTrajectoryVelocity();
      if (Util.epsilonEquals(newVel, mConstants.kCruiseVelocity, Math.max(1, mConstants.kMotionMagicDeadband)) || Util.epsilonEquals(newVel, mPeriodicIO.active_trajectory_velocity, Math.max(1, mConstants.kMotionMagicDeadband))) {
        // Mechanism is ~constant velocity.
        mPeriodicIO.active_trajectory_acceleration = 0.0;
      } else {
        // Mechanism is accelerating.
        mPeriodicIO.active_trajectory_acceleration = Math.signum(newVel - mPeriodicIO.active_trajectory_velocity) * mConstants.kAcceleration;
      }
      mPeriodicIO.active_trajectory_velocity = newVel;
    } else {
      mPeriodicIO.active_trajectory_position = Integer.MIN_VALUE;
      mPeriodicIO.active_trajectory_velocity = 0;
      mPeriodicIO.active_trajectory_acceleration = 0.0;
    }
    mPeriodicIO.error_ticks = (mLeader.getControlMode() == ControlMode.Position) ? mLeader.getClosedLoopError(0) : 0.0;

    mPeriodicIO.position_ticks = mLeader.getSelectedSensorPosition(0);
    mPeriodicIO.position_units = ticksToHomedUnits(mPeriodicIO.position_ticks);
    mPeriodicIO.velocity_ticks_per_100ms = mLeader.getSelectedSensorVelocity(0);

    if (mConstants.kRecoverPositionOnReset) {
      mPeriodicIO.absolute_pulse_position_modded = mLeader.getSensorCollection().getIntegratedSensorAbsolutePosition();
      if (mPeriodicIO.absolute_pulse_position_modded < 0.0) {
        mPeriodicIO.absolute_pulse_position_modded += mConstants.kLeaderConstants.ticksPerMotorRotation;
      }

      double estimated_pulse_pos = ((mConstants.kLeaderConstants.invert_sensor_phase ? -1.0 : 1.0) * mPeriodicIO.position_ticks) + mPeriodicIO.absolute_pulse_offset;
      double new_wraps = Math.floor(estimated_pulse_pos / ((double) mConstants.kLeaderConstants.ticksPerMotorRotation));
      // Only set this when we are really sure its a valid change
      if (Math.abs(mPeriodicIO.encoder_wraps - new_wraps) <= 1.0) {
        mPeriodicIO.encoder_wraps = new_wraps;
      }
    }
  }

  @Override
  public void updateHardware() {
    updateMotors();
  }

  // ------------------------------ SETTERS: SENSORS ------------------------------
  /** Hint: Return <b>false</b> when first writing subsystem */
  public abstract boolean atHomingLocation();

  public void resetIfAtHome() {
    if (atHomingLocation()) {
      zeroSensors();
    }
  }

  public void zeroSensors() {
    mLeader.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
    for (TalonFX talon : mFollowers) {
      talon.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
    }
    mPeriodicIO.absolute_pulse_offset = getAbsoluteEncoderRawPosition();
    mHasBeenZeroed = true;
  }

  public void forceZero() {
    mLeader.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
    for (TalonFX talon : mFollowers) {
      talon.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
    }
    mPeriodicIO.absolute_pulse_offset = getAbsoluteEncoderRawPosition();
  }

  // ------------------------------ SETTERS: RECOVERY ------------------------------
  public void recoverFromReset() {
    if (mPeriodicIO.reset_occured) {
      System.out.println(getName() + ": Leader Talon reset occurred; resetting frame rates.");
      mLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, mConstants.kStatusFrame2UpdateRate, 20);
      mLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, mConstants.kStatusFrame10UpdateRate, 20);
      mLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, mConstants.kStatusFrame8UpdateRate, 20);

      // Reset encoder position to estimated position from absolute encoder
      if (mConstants.kRecoverPositionOnReset) {
        mLeader.setSelectedSensorPosition(estimateSensorPositionFromAbsolute(), 0, Constants.CAN_TIMEOUT);
      }
    }
    for (TalonFX follower : mFollowers) {
      if (follower.hasResetOccurred()) {
        System.out.println(getName() + ": Follpower Talon reset occurred");
      }
    }
  }

  // ------------------------------ SETTERS: MOTORS ------------------------------
  public void setSetpointMotionMagic(double units, double feedforward_v) {
    mPeriodicIO.demand = constrainTicks(homedUnitsToTicks(units));
    mPeriodicIO.feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (mConstants.kMotionMagicPID.F + mConstants.kMotionMagicPID.D / 100.0) / 1023.0;
    setControlMode(ControlState.MOTION_MAGIC);
  }

  public void setSetpointPositionPID(double units, double feedforward_v) {
    mPeriodicIO.demand = constrainTicks(homedUnitsToTicks(units));
    mPeriodicIO.feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (mConstants.kMotionMagicPID.F + mConstants.kMotionMagicPID.D / 100.0) / 1023.0;
    setControlMode(ControlState.POSITION_PID);
  }

  public void setOpenLoop(double percentage) {
    mPeriodicIO.demand = percentage;
    mPeriodicIO.feedforward = 0.0;
    setControlMode(ControlState.OPEN_LOOP);
  }

  public void setBrakeMode(boolean enable) {
    if (isBrakeMode != enable) {
      NeutralMode mode = enable ? NeutralMode.Brake : NeutralMode.Coast;
      mLeader.setNeutralMode(mode);
      for (int i = 0; i < mFollowers.length; ++i) {
        mFollowers[i].setNeutralMode(mode);
      }
      isBrakeMode = enable;
    }
  }

  // ------------------------------ GETTERS ------------------------------
  public double estimateSensorPositionFromAbsolute() {
    double estimated_pulse_pos = (mPeriodicIO.encoder_wraps * mConstants.kLeaderConstants.ticksPerMotorRotation) + mPeriodicIO.absolute_pulse_position_modded;
    double estimate_position_ticks = (mConstants.kLeaderConstants.invert_sensor_phase ? -1.0 : 1.0) * (estimated_pulse_pos - mPeriodicIO.absolute_pulse_offset);
    return estimate_position_ticks;
  }

  public double getPredictedPositionUnits(double lookahead_secs) {
    if (mLeader.getControlMode() != ControlMode.MotionMagic) {
      return getPosition();
    }
    double predicted_units = ticksToHomedUnits(mPeriodicIO.active_trajectory_position + lookahead_secs * mPeriodicIO.active_trajectory_velocity + 0.5 * mPeriodicIO.active_trajectory_acceleration * lookahead_secs * lookahead_secs);
    if (mPeriodicIO.demand >= mPeriodicIO.active_trajectory_position) {
      return Math.min(predicted_units, ticksToHomedUnits(mPeriodicIO.demand));
    } 
    return Math.max(predicted_units, ticksToHomedUnits(mPeriodicIO.demand));
  }

  /** In "Units" */
  public double getPosition() { return ticksToHomedUnits(mPeriodicIO.position_ticks); }
  /** In "Units per second" */
  public double getVelocity() { return ticksToUnits(mPeriodicIO.velocity_ticks_per_100ms) * 10.0; }
  public double getSetpoint() { return (mControlState == ControlState.MOTION_MAGIC || mControlState == ControlState.POSITION_PID) ? ticksToUnits(mPeriodicIO.demand) : Double.NaN; }
  public double getSetpointHomed() { return (mControlState == ControlState.MOTION_MAGIC || mControlState == ControlState.POSITION_PID) ? ticksToHomedUnits(mPeriodicIO.demand) : Double.NaN; }
  public double getPositionTicks() { return mPeriodicIO.position_ticks; }
  public boolean hasBeenZeroed() { return mHasBeenZeroed; }
  /** @return absolute encoders raw ticks bounded to one rotation */
  protected double getAbsoluteEncoderRawPosition() {
    double abs_raw_with_rollover = mLeader.getSensorCollection().getIntegratedSensorAbsolutePosition();
    return abs_raw_with_rollover + (abs_raw_with_rollover < 0.0 ? abs_raw_with_rollover + mConstants.kLeaderConstants.ticksPerMotorRotation : 0.0);
  }

  // ------------------------------ UNIT CONVERSION ------------------------------
  protected double ticksToUnits(double ticks) { return ticks / mConstants.kTicksPerUnitDistance; }
  protected double unitsToTicks(double units) { return units * mConstants.kTicksPerUnitDistance; }
  protected double ticksToHomedUnits(double ticks) { return ticksToUnits(ticks) + mConstants.kHomePosition; }
  protected double homedUnitsToTicks(double units) { return unitsToTicks(units - mConstants.kHomePosition); }
  protected double ticksPer100msToUnitsPerSecond(double ticks_per_100ms) { return ticksToUnits(ticks_per_100ms) * 10.0; }
  protected double unitsPerSecondToTicksPer100ms(double units_per_second) { return unitsToTicks(units_per_second) / 10.0; }
  protected double constrainTicks(double ticks) { return Util.limit(ticks, mReverseSoftLimitTicks, mForwardSoftLimitTicks); }

  protected void setControlMode(ControlState mode) {
    if (mControlState == mode) {
      return;  // Already in this mode
    }
    if (mode != ControlState.OPEN_LOOP) {
      mLeader.selectProfileSlot(mode == ControlState.MOTION_MAGIC ? kMotionProfileSlot : kPositionPIDSlot, 0);
    }
    mControlState = mode;
  }

  protected void updateMotors() {
    TalonFXControlMode mode;
    if (mControlState == ControlState.MOTION_MAGIC) {
      mode = TalonFXControlMode.MotionMagic;
    } else if (mControlState == ControlState.POSITION_PID) {
      mode = TalonFXControlMode.Position;
    } else {
      mode = TalonFXControlMode.PercentOutput;
    }
    mLeader.set(mode, mPeriodicIO.demand, DemandType.ArbitraryFeedForward, mPeriodicIO.feedforward);
  }
}