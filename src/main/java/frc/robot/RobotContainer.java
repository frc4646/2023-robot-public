package frc.robot;

import java.util.ArrayList;
import java.util.List;

import frc.robot.commands.arms.ArmPositionTeleop;
import frc.robot.commands.diagnostics.SignalDriveTeam;
import frc.robot.commands.drive.DriveDisabled;
import frc.robot.commands.drive.DriveTeleop;
import frc.robot.commands.intake.IntakePulseIn;
import frc.robot.commands.stinger.CubeFlip;
import frc.robot.commands.vision.VisionFront;
import frc.robot.commands.wrist.WristTeleop;
import frc.robot.configuration.AutoModeSelector;
import frc.robot.configuration.DriverControlsSticks;
import frc.robot.configuration.IDriverControls;
import frc.robot.configuration.OperatorControls;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.Infrastructure;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Robotstate;
import frc.robot.subsystems.Stinger;
import frc.robot.subsystems.Tuner;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.team4646.Candle;
import frc.team4646.Canifier;
import frc.team4646.SmartSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public static Tuner TUNER;
    public static Infrastructure INFRASTRUCTURE;
    // Subsystems
    public static Canifier CANIFIER;
    public static Intake INTAKE;
    public static SwerveDriveSubsystem SWERVE;
    public static Vision BACK_VISION;
    public static Vision FRONT_VISION;
    public static ArmSubsystem ARMS;
    public static WristSubsystem WRIST;;
    public static Candle CANDLE;
    public static Robotstate STATE;
    public static Diagnostics DIAGNOSTICS;
    public static Stinger STINGER;

    private final List<SmartSubsystem> allSubsystems = new ArrayList<SmartSubsystem>();

    // Controllers and Operator Interface
    public static IDriverControls driver;
    public static OperatorControls operator;

    public AutoModeSelector autoModeSelector;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // create each subsystems
        INFRASTRUCTURE = new Infrastructure();
        CANIFIER = new Canifier(Constants.CAN.CANIFIER);
        SWERVE = new SwerveDriveSubsystem();
        INTAKE = new Intake();
        ARMS = new ArmSubsystem();
        WRIST = new WristSubsystem();
        BACK_VISION = new Vision("limelight-back");
        FRONT_VISION = new Vision("limelight-front");
        STINGER = new Stinger();
        CANDLE = new Candle(Constants.CAN.CANDLE);
        STATE = new Robotstate();
        DIAGNOSTICS = new Diagnostics();
        if(Constants.TUNING.CHARGE_STATION) TUNER = new Tuner();

        // add all subsystems to our list
        allSubsystems.add(INFRASTRUCTURE);
        allSubsystems.add(CANIFIER);
        allSubsystems.add(SWERVE);
        allSubsystems.add(INTAKE);
        allSubsystems.add(ARMS);
        allSubsystems.add(WRIST);
        allSubsystems.add(BACK_VISION);
        allSubsystems.add(FRONT_VISION);
        allSubsystems.add(STINGER);
        allSubsystems.add(CANDLE);
        allSubsystems.add(STATE);
        allSubsystems.add(DIAGNOSTICS);
        if(Constants.TUNING.CHARGE_STATION) allSubsystems.add(TUNER);

        // setup controls
        driver = new DriverControlsSticks();
        operator = new OperatorControls();

        // setup default commands for each subsystem
        SWERVE.setDefaultCommand(new DriveTeleop());//.until(() -> { return DriverStation.getMatchTime() < 0.5; }).andThen(new LockPatternDrive()));
        INTAKE.setDefaultCommand(new IntakePulseIn());//new IntakeBlocking(0.0));
        ARMS.setDefaultCommand(new ArmPositionTeleop());
        WRIST.setDefaultCommand(new WristTeleop());
        DIAGNOSTICS.setDefaultCommand(new SignalDriveTeam());
        FRONT_VISION.setDefaultCommand(new VisionFront());
        STINGER.setDefaultCommand(new CubeFlip(false));

        autoModeSelector = new AutoModeSelector();
    }

    public void cacheSensors() {
        allSubsystems.forEach(SmartSubsystem::cacheSensors);
    }

    public void updateHardware() {
        allSubsystems.forEach(SmartSubsystem::updateHardware);
    }

    public void onEnable(boolean isAutonomous) {
        allSubsystems.forEach(s -> s.onEnable(isAutonomous));
    }

    public void onDisable() {
        allSubsystems.forEach(SmartSubsystem::onDisable);
        new DriveDisabled().schedule();
    }

    public void whileDisabled() {
        allSubsystems.forEach(SmartSubsystem::whileDisabled);
    }

    public void runTests() {
        // if (VISION != null) {
        //     VISION.setLED(LEDMode.BLINK);
        // }
        // Test.reset();
        // Timer.delay(3.0);
        // if (VISION != null) {
        //     VISION.setLED(LEDMode.OFF);
        // }
        // allSubsystems.forEach(SmartSubsystem::runTests);
        // Test.results();
    }
}
