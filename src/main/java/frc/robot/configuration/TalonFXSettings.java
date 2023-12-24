package frc.robot.configuration;

import java.util.List;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.Pair;
import frc.robot.Constants;
import frc.team254.drivers.TalonFXFactory;
import frc.team4646.PID;
import frc.team4646.TalonUtil;

public class TalonFXSettings {
  private static final int TIMEOUT = Constants.CAN_TIMEOUT;

  public static TalonFX createMotor(TalonFXParams params) {
    TalonFX leader = TalonFXFactory.createDefaultTalon(params.idLeader, params.canbus);
    leader.setNeutralMode(NeutralMode.Brake);
    leader.setInverted(params.invert);
    leader.configNeutralDeadband(0.001);
    leader.configVoltageCompSaturation(12.0, TIMEOUT);
    leader.enableVoltageCompensation(true);

    for (int id : params.idFollowers) {
      TalonFX follower = TalonFXFactory.createPermanentFollowerTalon(id, params.canbus, leader);
      follower.setInverted(params.invert);
      follower.follow(leader);
      follower.configNeutralDeadband(0.001);  // TODO Added this untested - is it good?  
    }
    return leader;
  }

  public static CANCoder createEncoder(CANCoderParams params) {
    CANCoder encoder = new CANCoder(params.id, params.bus);
    encoder.configFactoryDefault();
    encoder.configAllSettings(params.config);
    return encoder;
  }

  public static void configureOpenLoop(TalonFX motor, double openloopRampRate) {
    motor.configOpenloopRamp(openloopRampRate, TIMEOUT);
  }

  public static void configureCurrentLimits(TalonFX motor, double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime) {
    SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(true, currentLimit, triggerThresholdCurrent, triggerThresholdTime);
    TalonUtil.checkError(motor.configSupplyCurrentLimit(limit), "Could not set supply current limit: " + motor.getDeviceID());
  }

  public static void configureEncoder(TalonFX motor) {
    motor.setSelectedSensorPosition(0, 0, TIMEOUT);
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, TIMEOUT);
    // TODO normally want limit switch frame higher if always reseting location when at home sensor
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 1000, TIMEOUT);  // Limit switches
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, TIMEOUT);  // Position
  }

  public static void configureRemoteEncoder(TalonFX motor, CANCoder encoder, boolean phase) {
    motor.setSensorPhase(phase);
    motor.configRemoteFeedbackFilter(encoder.getDeviceID(), RemoteSensorSource.CANCoder, 0, TIMEOUT);
    motor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0, TIMEOUT);
    motor.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
    // TODO normally want limit switch frame higher if always reseting location when at home sensor
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 1000, TIMEOUT);  // Limit switches
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, TIMEOUT);  // Position
  }

  public static void configureMotionMagic(TalonFX motor, MotionMagicParams params) {
    motor.selectProfileSlot(0, 0);

    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT);
    motor.configMotionCruiseVelocity(params.velocityCruiseEncoder, TIMEOUT);
    motor.configMotionAcceleration(params.accelerationEncoder, TIMEOUT);
    motor.configMotionSCurveStrength(params.sCurveStrength, TIMEOUT);

    TalonFXFactory.setPID(motor, params.pid);

    motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms, TIMEOUT);
    motor.configVelocityMeasurementWindow(1, TIMEOUT);
  }

  public static void configureHardwareLimits(TalonFX motor, HardwareLimitParams params) {
    if (params.enableR) {
      TalonUtil.checkError(motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen), "Could not set reverse limit switch for falcon " + motor.getDeviceID());
    }
    if (params.enableF) {
      TalonUtil.checkError(motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen), "Could not set forward limit switch for falcon " + motor.getDeviceID());
    }
    motor.overrideLimitSwitchesEnable(params.enableR || params.enableF);
  }

  public static void configureSoftwareLimits(TalonFX motor, SoftwareLimitParams params) {
    TalonUtil.checkError(motor.configReverseSoftLimitThreshold(params.nativeR, TIMEOUT), ": Could not set reverse soft limit: " + motor.getDeviceID());
    TalonUtil.checkError(motor.configReverseSoftLimitEnable(params.enableR, TIMEOUT), ": Could not enable reverse soft limit: " + motor.getDeviceID());
    TalonUtil.checkError(motor.configForwardSoftLimitThreshold(params.nativeF, TIMEOUT), ": Could not set forward soft limit: " + motor.getDeviceID());
    TalonUtil.checkError(motor.configForwardSoftLimitEnable(params.enableF, TIMEOUT), ": Could not enable forward soft limit: " + motor.getDeviceID());
    motor.overrideSoftLimitsEnable(false);  // Call this function later in your subsystem to actually enable (ex: after first time homing sensor is triggered)
  }

  // Support for optional builder pattern usage
  private final TalonFX motor;
  private final CANCoder encoder;
  public TalonFXSettings(TalonFXParams params, CANCoderParams paramsEncoder) {
    this.motor = createMotor(params);
    this.encoder = createEncoder(paramsEncoder);
  }
  public TalonFXSettings configureOpenLoop(double openloopRampRate) { TalonFXSettings.configureOpenLoop(motor, openloopRampRate); return this;}
  public TalonFXSettings configureCurrentLimits(double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime) { TalonFXSettings.configureCurrentLimits(motor, currentLimit, triggerThresholdCurrent, triggerThresholdTime); return this; }
  public TalonFXSettings configureEncoder() { TalonFXSettings.configureEncoder(motor); return this; }
  public TalonFXSettings configureRemoteEncoder(boolean phase) { TalonFXSettings.configureRemoteEncoder(motor, encoder, phase); return this; }
  public TalonFXSettings configureMotionMagic(MotionMagicParams params) { TalonFXSettings.configureMotionMagic(motor, params); return this; }
  public TalonFXSettings configureHardwareLimits(HardwareLimitParams params) { TalonFXSettings.configureHardwareLimits(motor, params); return this; }
  public TalonFXSettings configureSoftwareLimits(SoftwareLimitParams params) { TalonFXSettings.configureSoftwareLimits(motor, params); return this; }
  public Pair<TalonFX,CANCoder> get() { return new Pair<TalonFX,CANCoder>(motor, encoder); }

  // Configuration classes
  public static class TalonFXParams {
    public int idLeader;
    public String canbus;
    public TalonFXInvertType invert;
    public List<Integer> idFollowers;

    public TalonFXParams(int idLeader, String canbus, TalonFXInvertType invert, Integer... idFollowers) {
      this.idLeader = idLeader;
      this.canbus = canbus;
      this.invert = invert;
      this.idFollowers = List.of(idFollowers);
    }
  }

  public static class CANCoderParams {
    public int id;
    public String bus;
    public CANCoderConfiguration config;

    public CANCoderParams(int id, String bus, CANCoderConfiguration config) {
      this.id = id;
      this.bus = bus;
      this.config = config;
    }
  }

  public static class MotionMagicParams {
    public PID pid;
    public double velocityCruiseEncoder;
    public double accelerationEncoder;
    public int sCurveStrength;
    public double accelerationSeconds;
    
    public MotionMagicParams(PID pid, double velocityCruiseEncoder, double accelerationSeconds, int sCurveStrength) {
      this.pid = pid;
      this.velocityCruiseEncoder = velocityCruiseEncoder;
      this.accelerationEncoder = velocityCruiseEncoder / accelerationSeconds;
      this.sCurveStrength = sCurveStrength;
      this.accelerationSeconds = accelerationSeconds;
    }
  }

  public static class HardwareLimitParams {
    public boolean enableR, enableF;

    public HardwareLimitParams(boolean enableR, boolean enableF) {
      this.enableR = enableR;
      this.enableF = enableF;
    }
  }

  public static class SoftwareLimitParams {
    public boolean enableR, enableF;
    public double nativeR, nativeF;

    public SoftwareLimitParams(boolean enableR, boolean enableF, double nativeR, double nativeF) {
      this.enableR = enableR;
      this.enableF = enableF;
      this.nativeR = nativeR;
      this.nativeF = nativeF;
    }
  }
}
