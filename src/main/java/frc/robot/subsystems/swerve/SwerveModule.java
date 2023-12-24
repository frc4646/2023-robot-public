package frc.robot.subsystems.swerve;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.team4646.Util;

public class SwerveModule
{
    private final SwerveModuleConfig moduleConfig;
    private final SwerveMotorDrive driveMotor;
    private final SwerveMotorSteer steerMotor;

    private GenericEntry offsetWidget;
    private double lastOffset = 0.0;
    private GenericEntry saveOffsetButton;

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, and turning encoder.
     */
    public SwerveModule(SwerveModuleConfig config)
    {
        moduleConfig = config;

        driveMotor = new SwerveMotorDrive(moduleConfig);
        steerMotor = new SwerveMotorSteer(moduleConfig);
    }

    public void handleCalibration(boolean enabled) {
        steerMotor.handleCalibration(enabled);
    }

    public void forceCalibration() {
      steerMotor.calibrateOffsetAngle();
    }
    
    /**
     * calibrate the steering motor, update the angle offset from the dashboard
     */
    public void handleCalibrationWidget() {
        
        if (offsetWidget != null) {
            // set the offset to the manually specified angle
            steerMotor.setOffset(Rotation2d.fromDegrees(offsetWidget.getDouble(lastOffset)));
        }

        if (saveOffsetButton != null && saveOffsetButton.getBoolean(false)) {
            saveOffsetButton.setBoolean(false); // clear the button

            // set the offset to the current angle of the encoder
            steerMotor.setOffset(steerMotor.getRawEncoderAngle());
            offsetWidget.setDouble(steerMotor.getRawEncoderAngle().getDegrees());
        }
    }

    /**
     * Sets the desired state for the module.
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
    {
        desiredState = optimize(desiredState);
        steerMotor.setDesiredAngle(desiredState);
        driveMotor.setDesiredSpeed(desiredState, isOpenLoop);
    }

    /**
     * Optimize the commanded angle by keeping it within 360 degrees
     * and determining the shortest path to the desired angle
     * @param desiredState
     * @return optimized desiredState
     */
    private SwerveModuleState optimize(SwerveModuleState desiredState)
    {
        double currentAngle = steerMotor.getMotorAngle().getDegrees();
        double targetAngle = convertDesiredToMotorAngle(currentAngle, desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        
        double delta = targetAngle - currentAngle;

        // find a quick turn and invert the speed
        if (Math.abs(delta) > 90.0) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180); 
        }

        // Prevent rotating module if speed is less then 1%. Prevents Jittering
        if(Math.abs(targetSpeed) <= (Constants.SWERVE_DRIVE.MAX_SPEED_MPS * 0.01))
        {
            targetAngle = steerMotor.getLastDesiredAngle();
        }
        
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /**
     * find the optimum angle within 0 to 360 degrees
     * @param currentAngle current angle from the motor, -inf to +inf
     * @param targetAngle desired angle for the wheel, 0 to 360
     * @return shortest path angle for the motor
     */
    private static double convertDesiredToMotorAngle(double currentAngle, double targetAngle) {
        // find the current 0-360 angle, then figure out  
        double offset = currentAngle % 360;
        double lowerBounds = currentAngle - offset;
        double upperBounds = currentAngle + (360 - offset);

        // get it closer to current motor's rotation
        while (targetAngle < lowerBounds) {
            targetAngle += 360;
        }
        while (targetAngle > upperBounds) {
            targetAngle -= 360;
        }

        // actually put it within to +/- 180 of the motor
        if (targetAngle - currentAngle > 180) {
            targetAngle -= 360;
        } else if (targetAngle - currentAngle < -180) {
            targetAngle += 360;
        }

        return targetAngle;
    }
    
    /** Get the current state of the module for WPILIB calcs */
    public SwerveModuleState getState()
    {
        return new SwerveModuleState(driveMotor.getSpeed(), steerMotor.getMotorAngle());
    }

    /**
     * Put the module into brake or coast mode
     */
    public void setBrakeMode(boolean brakeModeEn) {
        steerMotor.setBrakeMode(brakeModeEn);
        driveMotor.setBrakeMode(brakeModeEn);
    }

    public TalonFX getSteerMotor() {
        return steerMotor.getMotor();
    }

    public TalonFX getDriveMotor() {
        return driveMotor.getMotor();
    }

    /**
     * Get the distance and angle
     * @return
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getDistance(), steerMotor.getMotorAngle());
    }

    /**
     * Create the basic Swerve Module dashboard
     */
    public void createDashboardGrid(ShuffleboardTab tab, int xPosition)
    {
        var layout = tab.getLayout(moduleConfig.name, BuiltInLayouts.kGrid)
                        .withSize(2, 4)
                        .withPosition(xPosition, 0)
                        .withProperties(Map.of("Number of rows", 5, "Number of columns", 2));

        // first row
        if(Constants.TUNING.SWERVE) {
            layout.addNumber("Desired Speed", () -> Util.round(driveMotor.getTargetSpeed(), 2)).withPosition(0, 0);
        }
        layout.addNumber("Current Speed", () -> Util.round(driveMotor.getSpeed(), 2)).withPosition(1, 0);

        // second row
        if(Constants.TUNING.SWERVE) {
            layout.addNumber("Desired Angle", () -> Util.round(steerMotor.getLastDesiredAngle(), 2)).withPosition(0, 1);
        }
        layout.addNumber("Motor Angle", () -> Util.round(steerMotor.getMotorAngle().getDegrees(), 2)).withPosition(1, 1);

        // third row
        layout.addNumber("Raw Enc Angle", () -> Util.round(steerMotor.getRawEncoderAngle().getDegrees(), 2)).withPosition(0, 2);
        layout.addNumber("Abs Enc Angle", () -> Util.round(steerMotor.getModuleAngle().getDegrees(), 2)).withPosition(1, 2);

        // fourth row
        offsetWidget = layout.add("Offset Angle", moduleConfig.offset.getDegrees()).withPosition(0,3).getEntry();
        layout.addBoolean("Is Initialzed", () -> steerMotor.isInitialized()).withPosition(1,3);

        // fifth row
        if(Constants.TUNING.SWERVE) {
            saveOffsetButton = layout.add("Save Offset", false).withPosition(0, 4).withWidget(BuiltInWidgets.kToggleButton).getEntry();
            layout.addNumber("Current Speed Native", () -> Util.round(driveMotor.getMotor().getSelectedSensorVelocity(), 2)).withPosition(1, 4);
        }
    }
}
