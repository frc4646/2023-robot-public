package frc.team254.drivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.team4646.PID;

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class SparkMaxFactory {
    /** Set closed-loop constants for motor controller. Is more concise than applying each constant individually. */
    public static void setPID(SparkMaxPIDController controller, PID PID) {
        controller.setP(PID.P);
        controller.setI(PID.I);
        controller.setD(PID.D);
        controller.setFF(PID.F);
    }

    public static class Configuration {
        public boolean BURN_FACTORY_DEFAULT_FLASH = true;
        public IdleMode NEUTRAL_MODE = IdleMode.kCoast;
        public boolean INVERTED = false;

        public int STATUS_FRAME_0_RATE_MS = 10;
        public int STATUS_FRAME_1_RATE_MS = 1000;
        public int STATUS_FRAME_2_RATE_MS = 1000;

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;

        public boolean ENABLE_VOLTAGE_COMPENSATION = false;
        public double NOMINAL_VOLTAGE = 12.0;
    }

    private static final Configuration kDefaultConfiguration = new Configuration();
    private static final Configuration kSlaveConfiguration = new Configuration();

    static {
        kSlaveConfiguration.STATUS_FRAME_0_RATE_MS = 1000;  // applied output, faults, sticky faults, is follower
        kSlaveConfiguration.STATUS_FRAME_1_RATE_MS = 1000;  // velocity, temperature, input voltage, current 
        kSlaveConfiguration.STATUS_FRAME_2_RATE_MS = 1000;  // position
    }

    // Create a CANTalon with the default (out of the box) configuration.
    public static CANSparkMax createDefaultSparkMax(int id) {
        return createDefaultSparkMax(id, false);
    }
    public static CANSparkMax createDefaultSparkMax(int id, boolean inverted) {
        final CANSparkMax sparkMax = createSparkMax(id, kDefaultConfiguration);
        sparkMax.setInverted(inverted);
        return sparkMax;
    }

    private static void handleCANError(int id, REVLibError error, String message) {
        if (error != REVLibError.kOk) {
            DriverStation.reportError( "Could not configure spark id: " + id + " error: " + error.toString() + " " + message, false);
        }
    }

    /** 
     * @param inverted Set the follower to output opposite of the leader
     */
    public static CANSparkMax createPermanentSlaveSparkMax(int id, CANSparkMax master, boolean inverted) {
        final CANSparkMax sparkMax = createSparkMax(id, kSlaveConfiguration);
        handleCANError(id, sparkMax.follow(master, inverted), "setting follower");
        return sparkMax;
    }

    public static CANSparkMax createSparkMax(int id, Configuration config) {
        // Delay for CAN bus bandwidth to clear up.
        Timer.delay(0.25);
        LazySparkMax sparkMax = new LazySparkMax(id);

        // sparkMax.restoreFactoryDefaults(config.BURN_FACTORY_DEFAULT_FLASH);
        sparkMax.restoreFactoryDefaults();

        handleCANError(id, sparkMax.setCANTimeout(200), "set timeout");
        sparkMax.set(0.0);

        handleCANError(id, sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, config.STATUS_FRAME_0_RATE_MS), "set status0 rate");
        handleCANError(id, sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, config.STATUS_FRAME_1_RATE_MS), "set status1 rate");
        handleCANError(id, sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, config.STATUS_FRAME_2_RATE_MS), "set status2 rate");

        sparkMax.clearFaults();

        handleCANError(id, sparkMax.setIdleMode(config.NEUTRAL_MODE), "set neutrual");
        handleCANError(id, sparkMax.setOpenLoopRampRate(config.OPEN_LOOP_RAMP_RATE), "set open loop ramp");
        handleCANError(id, sparkMax.setClosedLoopRampRate(config.CLOSED_LOOP_RAMP_RATE), "set closed loop ramp");

        if (config.ENABLE_VOLTAGE_COMPENSATION) {
            handleCANError(id, sparkMax.enableVoltageCompensation(config.NOMINAL_VOLTAGE), "voltage compensation");
        } else {
            handleCANError(id, sparkMax.disableVoltageCompensation(), "voltage compensation");
        }

        return sparkMax;
    }
}