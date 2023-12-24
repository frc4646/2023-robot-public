package frc.robot.subsystems.swerve;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.team4646.PIDTuner;
import frc.team4646.PIDTuner.DashboardConfig;
import frc.team4646.Util;

public class SwerveDriveDashboard {
    private static final String DASH_NAME = "Swerve";
    private static final String MODULES_DASH_NAME = "Swerve Modules";

    private ShuffleboardTab mainTab;
    private ShuffleboardTab modulesTab;
    
    private boolean drivetrainDashInitialized = false;
    private GenericEntry moduleDashEnableButton;
    private boolean moduleDashInitialized = false;
    private PIDTuner drivePIDTuner;
    private PIDTuner steerPIDTuner;

    private final SwerveDriveSubsystem subsystem;

    public SwerveDriveDashboard(SwerveDriveSubsystem subsystem) {
        this.subsystem = subsystem;
    }
    
    /**
     * Create a dashboard for the drivetrain basics
     */
    public void periodic() {
        createSubsystemDash();
        createModuleDash();
        updatePIDTunes();
    }

    /**
     * Create a dashboard for the drivetrain basics
     */
    public boolean createSubsystemDash() {
        if (!drivetrainDashInitialized && Constants.SWERVE_DRIVE.DASHBOARD_ENABLED) {
            drivetrainDashInitialized = true;
            mainTab = Shuffleboard.getTab(DASH_NAME);
            mainTab.getComponents().clear();

            mainTab.addBoolean("Field Relative", () -> subsystem.getFieldRelative()).withPosition(0, 0).withSize(2,2);
            if(Constants.TUNING.SWERVE) {
                mainTab.addNumber("Cmd: Rot", () -> Util.round(Units.radiansToDegrees(subsystem.getDesiredSpeeds().omegaRadiansPerSecond), 2)).withPosition(0, 2).withSize(2,1);
                mainTab.addNumber("Cmd: X Speed", () -> Util.round(subsystem.getDesiredSpeeds().vxMetersPerSecond, 2)).withPosition(0, 3).withSize(1,1);
                mainTab.addNumber("Cmd: Y Speed", () -> Util.round(subsystem.getDesiredSpeeds().vyMetersPerSecond, 2)).withPosition(1, 3).withSize(1,1);
            }
            mainTab.add("Gyro", subsystem.getGyro()).withPosition(5, 0).withSize(3,3); // set it be CCW+
            if(Constants.TUNING.SWERVE) {
                mainTab.addNumber("Gyro Yaw", () -> subsystem.getGyroAngle().getDegrees()).withPosition(4, 0).withSize(1,1);
                mainTab.addNumber("Gyro Pitch", () -> subsystem.getPitch().getDegrees()).withPosition(4, 1).withSize(1,1);
                mainTab.addNumber("Gyro Roll", () -> subsystem.getRoll().getDegrees()).withPosition(4, 2).withSize(1,1);
            }
            mainTab.addBoolean("Gyro Initialized", () -> subsystem.getGyroInitialized()).withPosition(3, 2).withSize(1,1);
            if(Constants.TUNING.SWERVE) {
                mainTab.addDouble("Pose: X", () -> subsystem.getOdometry().getEstimatedPosition().getX()).withPosition(3, 3).withSize(1,1);
                mainTab.addDouble("Pose: Y", () -> subsystem.getOdometry().getEstimatedPosition().getY()).withPosition(4, 3).withSize(1,1);
                mainTab.addDouble("Pose: Rot", () -> subsystem.getOdometry().getEstimatedPosition().getRotation().getDegrees()).withPosition(5, 3).withSize(1,1);
            }
            // mainTab.add ("Field", subsystem.field).withPosition(5, 0).withSize(5, 3);

            mainTab.add(subsystem).withPosition(8, 3);
            moduleDashEnableButton = mainTab.add("Enable Modules Tab", moduleDashInitialized).withPosition(6, 3).withSize(2,1).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        }

        return drivetrainDashInitialized;
    }

    /**
     * Create a dashboard for tuning amd calibration
     */
    public void createModuleDash() {
        if (moduleDashEnableButton != null && moduleDashEnableButton.getBoolean(false)) {
            moduleDashEnableButton.setBoolean(false); // clear the button

            if (!moduleDashInitialized) {
                moduleDashInitialized = true;
                
                modulesTab = Shuffleboard.getTab(MODULES_DASH_NAME);
                modulesTab.getComponents().clear();
        
                // add the 4 modules
                int x = 0;
                SwerveModule[] modules = subsystem.getSwerveModules();
                for (SwerveModule module : modules) {
                    module.createDashboardGrid(modulesTab, x);
                    x += 2;
                }
        
                // add the 2 tuners
                if (Constants.TUNING.SWERVE) {
                  drivePIDTuner = new PIDTuner(new DashboardConfig(MODULES_DASH_NAME, "Drive", x, 0), Constants.SWERVE_DRIVE.DRIVE_PID,
                          modules[0].getDriveMotor(),
                          modules[1].getDriveMotor(),
                          modules[2].getDriveMotor(),
                          modules[3].getDriveMotor());
                  x += 1;
                  steerPIDTuner = new PIDTuner(new DashboardConfig(MODULES_DASH_NAME, "Steer", x, 0), Constants.SWERVE_DRIVE.STEER_PID,
                          modules[0].getSteerMotor(),
                          modules[1].getSteerMotor(),
                          modules[2].getSteerMotor(),
                          modules[3].getSteerMotor());
                }
            }
        }
    }

    public void updatePIDTunes() {
        // ensure these exist first
        if(drivePIDTuner != null) drivePIDTuner.updateMotorPIDF();
        if(steerPIDTuner != null) steerPIDTuner.updateMotorPIDF();
    }

    public ShuffleboardTab getMainTab() {
        return mainTab;
    }

    public ShuffleboardTab getModulesTab() {
        return modulesTab;
    }
}
