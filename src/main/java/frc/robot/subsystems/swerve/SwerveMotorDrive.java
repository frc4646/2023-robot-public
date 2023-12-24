package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.team254.drivers.LazyTalonFX;
import frc.team254.util.Util;
import frc.team4646.TalonUtil;

/**
 * Wraps the Falcon motor for swerve drive propulsion
 */
public class SwerveMotorDrive
{
    private final SwerveModuleConfig moduleConfig;
    private final TalonFX motor;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SWERVE_DRIVE.DRIVE_KS, Constants.SWERVE_DRIVE.DRIVE_KV, Constants.SWERVE_DRIVE.DRIVE_KA);
    private double desiredSpeed;

    public SwerveMotorDrive(SwerveModuleConfig config) {
        moduleConfig = config;
        motor = new LazyTalonFX(moduleConfig.driveID, Constants.SWERVE_DRIVE.CAN_BUS_NAME);
        motor.configFactoryDefault();
        motor.configAllSettings(Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.driveConfig);
        motor.setInverted(Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.driveMotorInvert);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT);
        motor.enableVoltageCompensation(true);
        motor.setSelectedSensorPosition(0.0);
    }

    /**
     * Set the desried speed
     * @param desiredState
     * @param isOpenLoop if true, use percent output mode. if false, use velocity mode
     */
    public void setDesiredSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        double setpoint;
        double arbFeedForward = 0.0;
        ControlMode mode;            
        
        if(isOpenLoop) {
            setpoint = desiredState.speedMetersPerSecond / Constants.SWERVE_DRIVE.MAX_SPEED_MPS;
            mode = ControlMode.PercentOutput;
            if(!Util.epsilonEquals(setpoint, .05)) {
                arbFeedForward = Constants.SWERVE_DRIVE.DRIVE_ARB_FEEDFORWARD * Math.signum(setpoint);
            }
        }
        else {
            setpoint = TalonUtil.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.wheelCircumference, Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.driveGearRatio);
            mode = ControlMode.Velocity;
        }


        motor.set(mode, setpoint, DemandType.ArbitraryFeedForward, arbFeedForward);

        desiredSpeed = desiredState.speedMetersPerSecond;
    }

    /**
     * Get the desired target speed
     * @return
     */
    public double getTargetSpeed() {
        return desiredSpeed;
    }

    /**
     * Get the current speed as reported by the motor in mps
     * @return 
     */
    public double getSpeed() {
        return TalonUtil.falconToMPS(motor.getSelectedSensorVelocity(), Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.wheelCircumference, Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.driveGearRatio);
    }

    /**
     * Get the distance travelled of the wheel in meters
     * @return
     */
    public double getDistance() {
        return TalonUtil.falconToMeters(motor.getSelectedSensorPosition(), Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.wheelCircumference, Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.driveGearRatio);
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

