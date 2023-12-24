package frc.team4646;

import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * parts from https://github.com/Team364/BaseFalconSwerve/
 */
public class TalonUtil {
    public static final double TICKS_PER_ROTATION = 2048.0;
    public static final double CANCODER_TICKS_PER_ROTATION = 4096.0;
    
    
    /**
     * Checks the specified error code for issues
     * @param errorCode error code
     * @param message   message to print if error happens
     * @return true if success, false if error
     */
    public static boolean checkError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            DriverStation.reportError(message + errorCode, false);
            return false;
        }
        return true;
    }

    /**
     * @param counts CANCoder Counts
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double CANcoderToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * CANCODER_TICKS_PER_ROTATION));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return CANCoder Counts
     */
    public static double degreesToCANcoder(double degrees, double gearRatio) {
        double ticks =  degrees / (360.0 / (gearRatio * CANCODER_TICKS_PER_ROTATION));
        return ticks;
    }

    /**
     * @param counts Falcon Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * TICKS_PER_ROTATION));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Counts
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        double ticks =  degrees / (360.0 / (gearRatio * TICKS_PER_ROTATION));
        return ticks;
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / TICKS_PER_ROTATION);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (TICKS_PER_ROTATION / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return MPS of Wheel
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    public static double falconToMeters(double counts, double circumference, double gearRatio) {
        return counts * (circumference / (gearRatio * TICKS_PER_ROTATION));
    }
}