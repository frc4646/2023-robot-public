package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.util.Units;
import frc.team4646.PID;

public class SDSModuleConfig {
    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double angleGearRatio;
    public final double driveGearRatio;
    public final boolean driveMotorInvert;
    public final boolean angleMotorInvert;
    
    public final CANCoderConfiguration encoderConfig;
    public final TalonFXConfiguration steerConfig;
    public final TalonFXConfiguration driveConfig;

    public SDSModuleConfig(double wheelDiameter, double angleGearRatio, double driveGearRatio, boolean driveMotorInvert, boolean angleMotorInvert, boolean canCoderInvert, PID drivePID, PID steerPID) {
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.driveMotorInvert = driveMotorInvert;
        this.angleMotorInvert = angleMotorInvert;

        encoderConfig = new CANCoderConfiguration();
        encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        encoderConfig.sensorDirection = canCoderInvert;
        encoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        encoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        
        steerConfig = new TalonFXConfiguration();
        steerConfig.slot0.kP = steerPID.P;
        steerConfig.slot0.kI = steerPID.I;
        steerConfig.slot0.kD = steerPID.D;
        steerConfig.slot0.kF = steerPID.F;
        steerConfig.neutralDeadband = .001;
        steerConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 25, 40, 0.1);
        
        driveConfig = new TalonFXConfiguration();
        driveConfig.slot0.kP = drivePID.P;
        driveConfig.slot0.kI = drivePID.I;
        driveConfig.slot0.kD = drivePID.D;
        driveConfig.slot0.kF = drivePID.F;        
        driveConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1);
        driveConfig.openloopRamp = 0.25;
        driveConfig.closedloopRamp = 0.0;
    }

    public static SDSModuleConfig SDSMK4i(double driveGearRatio, PID drivePID, PID steerPID){
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = ((150.0 / 7.0) / 1.0);

        boolean driveMotorInvert = true;
        boolean angleMotorInvert = true;
        boolean canCoderInvert = false;
        return new SDSModuleConfig(wheelDiameter, angleGearRatio, driveGearRatio, driveMotorInvert, angleMotorInvert, canCoderInvert, drivePID, steerPID);
    }

    /* Drive Gear Ratios for Mk4i */
    public class Mk4i{
        /** SDS MK4i - 8.14 : 1 */
        public static final double L1 = (50.0 / 14.0 * 19.0 / 25.0 * 45.0 / 15.0);
        /** SDS MK4i - 6.75 : 1 */
        public static final double L2 = (50.0 / 14.0 * 17.0 / 27.0 * 45.0 / 15.0);
        /** SDS MK4i - 6.12 : 1 */
        public static final double L3 = (50.0 / 14.0 * 16.0 / 28.0 * 45.0 / 15.0);
    }
}
