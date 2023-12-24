// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.team4646.SmartSubsystem;

/** Represents a swerve drive style drivetrain. */
public class SwerveDriveSubsystem extends SmartSubsystem {

    public static Field2d field = new Field2d();

    private final SwerveModule[] modules = new SwerveModule[] {
        new SwerveModule(Constants.SWERVE_DRIVE.MODULE_CONFIG[0]),
        new SwerveModule(Constants.SWERVE_DRIVE.MODULE_CONFIG[1]),
        new SwerveModule(Constants.SWERVE_DRIVE.MODULE_CONFIG[2]),
        new SwerveModule(Constants.SWERVE_DRIVE.MODULE_CONFIG[3]) 
    };

    private final AHRS gyro = new AHRS();
    private boolean gyroInitialized = false;

    private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(Constants.SWERVE_DRIVE.KINEMATICS, getGyroAngle(), getModulePositions(), new Pose2d());
    
    private boolean fieldRelative;
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    
     private final LinearFilter filter;

    private final SwerveDriveDashboard dashboard;

    private GenericEntry fixSwerveButton;

    public SwerveDriveSubsystem() {
        dashboard = new SwerveDriveDashboard(this);
        
        filter = LinearFilter.movingAverage(15);
        SmartDashboard.putData(field); //TODO figure out how to put this on a tab


        fixSwerveButton = Shuffleboard.getTab(Constants.COMP_DASH_NAME).add("Fix Swerve Modules", false)
            .withPosition(6, 3).withSize(2,1).withWidget(BuiltInWidgets.kToggleButton).getEntry();

    }

    @Override
    public void whileDisabled() {
        for (SwerveModule module : modules) {
            module.handleCalibration(false);
        }
    }
    
    /**
     * Handles updating changing steering angle offsets, PID tuning, gyro initialization, and odometry.
     * Gets ran before commands run, see {@link edu.wpi.first.wpilibj2.command.CommandScheduler#run()}
     */
    @Override
    public void periodic() {
        for (SwerveModule module : modules) {
            module.handleCalibrationWidget();
        }

        if(fixSwerveButton != null && fixSwerveButton.getBoolean(false)) {
            for (SwerveModule module : modules) {
                module.forceCalibration();
            }
        }
        
        // wait for the gyro to be detected before initializing once
        if (!gyroInitialized && gyro.isConnected()) {
            resetGyro();
            gyroInitialized = true;
        }

        updateOdometry();
        field.setRobotPose(odometry.getEstimatedPosition());

        dashboard.periodic();
    }

    @Override
    public void onEnable(boolean isAutonomous) {
      filter.reset();
    }

    /**
     * Drive the robot using joystick info.
     * @param xSpeed   Speed of the robot in the x direction (forward), in mps.
     * @param ySpeed   Speed of the robot in the y direction (sideways), in mps.
     * @param rotationDps angular rate of the robot, in deg/sec.
     * @param fieldRel Whether the provided x and y speeds are relative to the field.
     * @param openLoopDrive Whether the provided speeds in velocity control or in open loop
     */
    public void drive(Translation2d speedsMps, double rotationDps, boolean fieldRel, boolean openLoopDrive) {
        drive(speedsMps, rotationDps, fieldRel, openLoopDrive, new Translation2d());
    }
    
    /**
     * Drive the robot using joystick info.
     * @param xSpeed   Speed of the robot in the x direction (forward), in mps.
     * @param ySpeed   Speed of the robot in the y direction (sideways), in mps.
     * @param rotationDps angular rate of the robot, in deg/sec.
     * @param fieldRel Whether the provided x and y speeds are relative to the field.
     * @param openLoopDrive Whether the provided speeds in velocity control or in open loop
     * @param centerOfRotationMeters XY center of rotation
     */
    public void drive(Translation2d speedsMps, double rotationDps, boolean fieldRel, boolean openLoopDrive, Translation2d centerOfRotationMeters) {
        // convert the x/y/rot inputs to actual chassis commands depending on field relativity
        fieldRelative = fieldRel;
        if (fieldRelative) {
            desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedsMps.getX(), speedsMps.getY(), Math.toRadians(rotationDps), getGyroAngle());
        } else {
            desiredSpeeds = new ChassisSpeeds(speedsMps.getX(), speedsMps.getY(), Math.toRadians(rotationDps));
        }

        // use trig to calculate and set each module's desired states
        setDesiredStates(Constants.SWERVE_DRIVE.KINEMATICS.toSwerveModuleStates(desiredSpeeds, centerOfRotationMeters), openLoopDrive);
    }
    

    public void setChassisSpeeds(ChassisSpeeds speeds) {
      desiredSpeeds = speeds;
      setDesiredStates(Constants.SWERVE_DRIVE.KINEMATICS.toSwerveModuleStates(desiredSpeeds, new Translation2d()), false);
    }

    /**
     * Directly set the states of each module
     */
    public void setDesiredStates(SwerveModuleState[] desiredStates, boolean openLoopDrive) {
        // scale back the speeds to our max achievable
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SWERVE_DRIVE.MAX_SPEED_MPS);

        // set each module's desired state
        for (int i = 0; i < desiredStates.length; i++) {
            modules[i].setDesiredState(desiredStates[i], openLoopDrive);
        }
    }

    /**
     * Directly set the states of each module, in closed loop speed mode
     */
    public void setDesiredStates(SwerveModuleState[] desiredStates) {
        setDesiredStates(desiredStates, false);
    }

    /**
     * Reset the gyro angle to our current angle and the odometry to the new pose
     */
    public void resetGyroAndOdometry(Pose2d pose) {
        gyro.reset();
        resetOdometry(pose);
    }

    /**
     * Reset the gyro angle to our current position
     */
    public void resetGyro() {
        gyro.reset();
        resetOdometry(new Pose2d()); // replace new Pose2d with starting location in the field
    }

    /**
     * Reset the odometry to a new starting pose
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getGyroAngle(), getModulePositions(), pose);
    }

    /**
     * Updates the field relative position of the robot.
     */
    public void updateOdometry() {
        odometry.update(
            getGyroAngle(), // this may need to be inverted
            getModulePositions());

        if(RobotContainer.BACK_VISION.foundAprilTag() && getAveragedSpeedMps() < 0.1 ) {
        // if(RobotContainer.BACK_VISION.foundAprilTag()) {
            Pose2d backPose = RobotContainer.BACK_VISION.getRobotPositionFromApriltag().pose;
        
            if (backPose.minus(getPose()).getTranslation().getNorm() <= 0.5) {
                double latencyTimestamp =  RobotContainer.BACK_VISION.getRobotPositionFromApriltag().latency / 1000;
                odometry.addVisionMeasurement(backPose, Timer.getFPGATimestamp() - latencyTimestamp);
            }
        }
    }

    /**
     * Get the distance traveled of the modules for odometry purposes
     * @return
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    /**
     * Get the speed and angles of the modules
     * @return
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }
    
    /**
     * Get the current speed/angle of the chassis using forward kinematics
     * @return
     */
    public ChassisSpeeds getChassisSpeeds() {
        return Constants.SWERVE_DRIVE.KINEMATICS.toChassisSpeeds(getModuleStates());
    }
    
    /**
     * Put the module into brake or coast mode
     */
    public void setBrakeMode(boolean brakeModeEn) {
        for (SwerveModule module : modules) {
            module.setBrakeMode(brakeModeEn);
        }
    }

    public double getAveragedSpeedMps() {
        ChassisSpeeds speeds = getChassisSpeeds();
        double robotMetersPerSecond = Math.sqrt(speeds.vxMetersPerSecond * speeds.vxMetersPerSecond + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond);

        return filter.calculate(robotMetersPerSecond);
    }

    public boolean getFieldRelative() { return fieldRelative; }
    public ChassisSpeeds getDesiredSpeeds() { return desiredSpeeds; }
    public AHRS getGyro() { return gyro; }
    public boolean getGyroInitialized() { return gyroInitialized; }
    public Field2d getField() { return field; }
    public SwerveDrivePoseEstimator getOdometry() { return odometry; }
    public SwerveModule[] getSwerveModules() { return modules; }
    public Pose2d getPose() { return odometry.getEstimatedPosition(); }
    

    /**
     * Get the gyro angle in the correct orientation to the robot
     * @return Gyro angle, CCW+
     */
    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }
    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }
}