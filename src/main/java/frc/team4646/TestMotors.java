package frc.team4646;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team254.drivers.LazySparkMax;
import frc.team254.drivers.LazyTalonFX;
import frc.team254.util.Util;

/** Motor Test System */
public class TestMotors {
  public static class MotorConfig<T> {
    public String name;
    public T motor;

    public MotorConfig(String name, T motor) { this.name = name; this.motor = motor; }
    public MotorConfig(T motor) { this("", motor); }
  }

  /** Motor Test Parameter Builder */
  public static class TestConfig {
    public double percent = 0.5;
    public double timeRun = 4.0;
    public double timeWait = 2.0;

    public double ampsMin = 5.0;
    public double ampsEpsilon = 5.0;

    public double rpmMin = 2000.0;
    public double rpmEpsilon = 500.0;
    public DoubleSupplier rpmSupplier = null;

    public TestConfig test(double percent, double timeRun, double timeWait) { this.percent = percent; this.timeRun = timeRun; this.timeWait = timeWait; return this; }
    public TestConfig amps(double min, double epsilon) { ampsMin = min; ampsEpsilon = epsilon; return this; }
    public TestConfig rpm(double min, double epsilon, DoubleSupplier supplier) { rpmMin = min; rpmEpsilon = epsilon; rpmSupplier = supplier; return this; }
  }

  /** Motor Test Runner */
  public static abstract class MotorTest<T> {
    protected void checkMotorsImpl(SubsystemBase subsystem, List<MotorConfig<T>> configMotors, TestConfig configTest) {
      ArrayList<Double> currents = new ArrayList<>(), rpms = new ArrayList<>();
  
      motors = configMotors;
      storeConfig();
      configMotors.forEach(config -> setOpenLoop(config.motor, 0.0));
  
      for (MotorConfig<T> config : configMotors) {
        setOpenLoop(config.motor, configTest.percent);
        Timer.delay(configTest.timeRun);
  
        double current = getAmps(config.motor);
        double rpm = Double.NaN;
  
        currents.add(current);
        if (configTest.rpmSupplier != null) { rpm = configTest.rpmSupplier.getAsDouble(); rpms.add(rpm); }
  
        setOpenLoop(config.motor, 0.0);
        analyzeSingle(subsystem, configTest, config.name, current, rpm);      
        Timer.delay(configTest.timeWait);
      }
      analyzeGroup(subsystem, configTest, currents, rpms);
      restoreConfig();
    }
  
    private void analyzeSingle(SubsystemBase subsystem, TestConfig config, String motor, double current, double rpm) {
      System.out.print(String.format("%s: %s %.1f Amps", subsystem.getName(), motor, current));
      if (config.rpmSupplier != null) {
        System.out.println(String.format(", %.0f RPM", rpm));
      }
      Test.add(subsystem, String.format("%s Min Amps %.0f", motor, config.ampsMin), current < config.ampsMin);
      if (config.rpmSupplier != null) {
        Test.add(subsystem, String.format("%s Min RPM %.0f", motor, config.rpmMin), rpm < config.rpmMin);
      }
    }

    private void analyzeGroup(SubsystemBase subsystem, TestConfig config, List<Double> currents, List<Double> rpms) {
      if (currents.size() > 0) {
        double average = currents.stream().mapToDouble(val -> val).average().getAsDouble();
        Test.add(subsystem, "Similar Amps", Util.allCloseTo(currents, average, config.ampsEpsilon));
      }
      if (rpms.size() > 0) {
        double average = rpms.stream().mapToDouble(val -> val).average().getAsDouble();
        Test.add(subsystem, "Similar RPM", Util.allCloseTo(rpms, average, config.rpmEpsilon));
      }
    }

    protected List<MotorConfig<T>> motors;
    protected abstract void storeConfig();
    protected abstract void restoreConfig();
    protected abstract void setOpenLoop(T motor, double output);
    protected abstract double getAmps(T motor);
  }

  /** Motor Test Runner TalonFX-specifics */
  public static class TalonFXChecker extends MotorTest<BaseTalon> {
    private static class StoredTalonFXConfig {
      public ControlMode mode;
      public double setValue;
    }

    protected ArrayList<TalonFXChecker.StoredTalonFXConfig> storedConfigs = new ArrayList<>();

    public static void checkMotors(SubsystemBase subsystem, List<MotorConfig<BaseTalon>> motorsToCheck, TestConfig checkerConfig) {
      TalonFXChecker checker = new TalonFXChecker();
      checker.checkMotorsImpl(subsystem, motorsToCheck, checkerConfig);
    }

    @Override
    protected void storeConfig() {
      for (MotorConfig<BaseTalon> config : motors) {
        LazyTalonFX talon = (LazyTalonFX) config.motor;
        TalonFXChecker.StoredTalonFXConfig storedConfig = new TalonFXChecker.StoredTalonFXConfig();
        
        storedConfig.mode = talon.getControlMode();
        storedConfig.setValue = talon.getLastSet();
        storedConfigs.add(storedConfig);
      }
    }

    @Override
    protected void restoreConfig() {
      for (int i = 0; i < motors.size(); ++i) {
        motors.get(i).motor.set(storedConfigs.get(i).mode, storedConfigs.get(i).setValue);
      }
    }

    @Override
    protected void setOpenLoop(BaseTalon motor, double output) { motor.set(ControlMode.PercentOutput, output); }
    @Override
    protected double getAmps(BaseTalon motor) { return motor.getSupplyCurrent(); }
  }

  /** Motor Test Runner SparkMax-specifics */
  public static class MotorTestSparkMax extends MotorTest<CANSparkMax> {
    private static class StoredSparkConfig {
      CANSparkMax leader = null;
    }
    protected ArrayList<StoredSparkConfig> storedConfigs = new ArrayList<>();

    public static void checkMotors(SubsystemBase subsystem, List<MotorConfig<CANSparkMax>> motorsToCheck, TestConfig checkerConfig) {
      MotorTestSparkMax checker = new MotorTestSparkMax();
      checker.checkMotorsImpl(subsystem, motorsToCheck, checkerConfig);
    }

    @Override
    protected void storeConfig() {
      for (MotorConfig<CANSparkMax> config : motors) {
        LazySparkMax motor = (LazySparkMax) config.motor;
        StoredSparkConfig storedConfig = new StoredSparkConfig();
        
        storedConfig.leader = motor.getLeader();
        storedConfigs.add(storedConfig);
        motor.restoreFactoryDefaults();
      }
    }

    @Override
    protected void restoreConfig() {
      for (int i = 0; i < motors.size(); ++i) {
        if (storedConfigs.get(i).leader != null) {
          motors.get(i).motor.follow(storedConfigs.get(i).leader);
        }
      }
    }

    @Override
    protected void setOpenLoop(CANSparkMax motor, double output) { motor.getPIDController().setReference(output, CANSparkMax.ControlType.kDutyCycle);}
    @Override
    protected double getAmps(CANSparkMax motor) { return motor.getOutputCurrent(); }
  }
}
