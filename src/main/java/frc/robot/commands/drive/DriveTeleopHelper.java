// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveTeleopHelper {
    // Slew rate limiters to make joystick inputs more gentle lower value takes longer to get to the max
    private static final SlewRateLimiter
        m_xspeedLimiter = new SlewRateLimiter(Constants.SWERVE_DRIVE.MAX_ACCEL_MPSS),
        m_yspeedLimiter = new SlewRateLimiter(Constants.SWERVE_DRIVE.MAX_ACCEL_MPSS),
        m_rotLimiter = new SlewRateLimiter(Constants.SWERVE_DRIVE.MAX_ROTATION_ACCEL_DPSS);
    
    private static double getSpeedPercent() {
        double speedPercent = Constants.SWERVE_DRIVE.SPEED_PERCENT_NORMAL;
            
        if(RobotContainer.driver.getSpeedBoost()) {
            speedPercent = Constants.SWERVE_DRIVE.SPEED_PERCENT_FAST;
        } else if(RobotContainer.driver.getSpeedSlow()) {
            speedPercent = Constants.SWERVE_DRIVE.SPEED_PERCENT_SLOW;
        }
        return speedPercent;
    }

    public static Translation2d getMove(){
        double speedPercent = getSpeedPercent();
    
        double xSpeed = m_xspeedLimiter.calculate(RobotContainer.driver.getX() * speedPercent * Constants.SWERVE_DRIVE.MAX_SPEED_MPS);
        double ySpeed = m_yspeedLimiter.calculate(RobotContainer.driver.getY() * speedPercent * Constants.SWERVE_DRIVE.MAX_SPEED_MPS);

        return new Translation2d(xSpeed, ySpeed);

    }

    public static double getRotationRate() {
        double rotationRate = m_rotLimiter.calculate(RobotContainer.driver.getRotation() * getSpeedPercent() * Constants.SWERVE_DRIVE.MAX_ROTATION_SPEED_DPS);
        SmartDashboard.putNumber("Rotation Rate", rotationRate);
        return rotationRate;
    }
}
