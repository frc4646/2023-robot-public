package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.team254.drivers.LazyTalonFX;
import frc.team4646.TalonUtil;

/**
 * Wraps the Falcon motor and CANcoder for swerve drive turning
 */
public class SwerveMotorSteer
{
    private final SwerveModuleConfig moduleConfig;
    private final TalonFX motor;
    private final CANCoder encoder;
    private final double timeAtBootSec;

    private boolean initialized = false;
    private double resetIteration = 0;

    private double lastDesiredAngle;
    private boolean wasEnabled;

    public SwerveMotorSteer(SwerveModuleConfig config)
    {
        moduleConfig = config;

        encoder = new CANCoder(moduleConfig.cancoderID, Constants.SWERVE_DRIVE.CAN_BUS_NAME);
        encoder.configFactoryDefault();
        encoder.configAllSettings(Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.encoderConfig);

        motor = new LazyTalonFX(moduleConfig.turnID, Constants.SWERVE_DRIVE.CAN_BUS_NAME);
        motor.configFactoryDefault();
        motor.configAllSettings(Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.steerConfig);
        motor.setInverted(Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.angleMotorInvert);
        motor.setNeutralMode(NeutralMode.Coast);
        motor.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT);
        motor.enableVoltageCompensation(true);

        if (Constants.SWERVE_DRIVE.STEER_USE_REMOTE_SENSOR_FEATURE) {
            // TODO connect encoder directly to motor using the remote feedback feature
            motor.configRemoteFeedbackFilter(encoder, 0, Constants.CAN_TIMEOUT);
            motor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0, Constants.CAN_TIMEOUT);
        }

        timeAtBootSec = Timer.getFPGATimestamp();
        wasEnabled = false;
    }

    /**
     * Will calibrate the encoder with the stored offset angle when it's available.
     * Call from the subsystem's periodic function.
     */
    public void handleCalibration(boolean enabled) {
        // if enabled before it calibrated, keep trying
        // if(!initialized && (Timer.getFPGATimestamp() - timeAtBootSec) > 1) {
        // keep retrying until we enable once
        if(!wasEnabled) {
            if (++resetIteration >= 500) {
                resetIteration = 0;
                if(calibrateOffsetAngle()) {
                    // this will let first enable not rotate to 0 degrees
                    lastDesiredAngle = getModuleAngle().getDegrees();
                    initialized = true;
                }
            }
        }

        if (Constants.SWERVE_DRIVE.STEER_IDLE_RESYNC_ENCODER) {// || !enabled) {
            // automatic recalibration if sitting there idle
            if (TalonUtil.falconToRPM(motor.getSelectedSensorVelocity(), Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.angleGearRatio) < .1) {
                if (++resetIteration >= 500) {
                    resetIteration = 0;
                    calibrateOffsetAngle();
                }
            } else {
                resetIteration = 0;
            }
        }

        if(enabled) {
            wasEnabled = true;
        }
    }
    
    /**
     * Set the desired angle
     * @param desiredState
     */
    public void setDesiredAngle(SwerveModuleState desiredState) {
        lastDesiredAngle = desiredState.angle.getDegrees();

        if (Constants.SWERVE_DRIVE.STEER_USE_REMOTE_SENSOR_FEATURE) {
            // TODO connect encoder directly to motor using the remote feedback feature
            motor.set(ControlMode.Position, TalonUtil.degreesToCANcoder(lastDesiredAngle, 1));
        } else {
            motor.set(ControlMode.Position, TalonUtil.degreesToFalcon(lastDesiredAngle, Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.angleGearRatio));
        }
    }

    /**
     * Get the angle of the wheel as reported by the motor. Should be 0 when everything is calibrated.
     * Will increment/decrement past 360 as the wheel completes a full revolution.
     * @return -inf to +inf angle of the wheel
     */
    public Rotation2d getMotorAngle() {
        return Rotation2d.fromDegrees(TalonUtil.falconToDegrees(motor.getSelectedSensorPosition(), Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.angleGearRatio));
    }

    /**
     * Get the absoulte angle as reported by the encoder. Used to calibrate modules to 0 degrees.
     * Does not account for the offset angle
     * Wraps around at 360 degrees.
     * @return 0 to 360 angle of the wheel
     */
    public Rotation2d getRawEncoderAngle() {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
    }

    /**
     * Get the absoulte angle as reported by the encoder, with the offset angle applied.
     * Wraps around at 360 degrees.
     * @return 0 to 360 angle of the wheel
     */
    public Rotation2d getModuleAngle() {
        return getRawEncoderAngle().minus(moduleConfig.offset);
    }

    /**
     * Send the calibration value to the motor for aligning it to the encoder
     * @return true if position was written successfully
     */
    public boolean calibrateOffsetAngle() {
        // get the angle, 0 to 360, of what the encoder thinks our current angle is
        double angleWithOffset = getModuleAngle().getDegrees();
        // calculate what that angle is in Talon units
        double absoluteMotorPosition = TalonUtil.degreesToFalcon(angleWithOffset, Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.angleGearRatio);
        // send it to the motor to seed the initial position
        return TalonUtil.checkError(motor.setSelectedSensorPosition(absoluteMotorPosition, 0, Constants.CAN_TIMEOUT), moduleConfig.name + " Swerve Cal Failed");
    }

    /**
     * If the initial calibration offset was sent to the motor.
     * @return
     */
    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Set a new offset angle
     * @param newAngle
     */
    public void setOffset(Rotation2d newAngle) {
        // only store if the offset has changed
        if(!newAngle.equals(moduleConfig.offset)) {
            moduleConfig.offset = newAngle;
            calibrateOffsetAngle();
        }
    }

    /**
     * Get the last angle that was set
     * @return
     */
    public double getLastDesiredAngle() {
        return lastDesiredAngle;
    }

    /**
     * Put the motor into brake or coast mode
     */
    public void setBrakeMode(boolean brakeModeEn) {
        if (brakeModeEn) {
            motor.setNeutralMode(NeutralMode.Brake);
        } else {
            motor.setNeutralMode(NeutralMode.Coast);
        }
    }

    public TalonFX getMotor() {
        return motor;
    }
}
