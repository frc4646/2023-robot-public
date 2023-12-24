package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.configuration.TalonFXSettings.CANCoderParams;
import frc.robot.configuration.TalonFXSettings.HardwareLimitParams;
import frc.robot.configuration.TalonFXSettings.MotionMagicParams;
import frc.robot.configuration.TalonFXSettings.SoftwareLimitParams;
import frc.robot.configuration.TalonFXSettings.TalonFXParams;
import frc.robot.subsystems.swerve.SDSModuleConfig;
import frc.robot.subsystems.swerve.SwerveModuleConfig;
import frc.robot.util.ArmEnvelope;
import frc.robot.util.ArmState;
import frc.robot.util.Servo;
import frc.team4646.DiagnosticState;
import frc.team4646.EncoderMath;
import frc.team4646.LEDColor;
import frc.team4646.PID;

public final class Constants {
  public static int CAN_TIMEOUT = 100;
  public static final double FRAME_PERIMETER_X_INCHES = 26;
  public static final double FRAME_PERIMETER_Y_INCHES = 34;
  public static final double LOOP_TIME = 0.02;

  public static final String COMP_DASH_NAME = "Main";

  public static final class CAN {
    public static final String BUS_NAME = "rio"; // "rio" is built-in
    
    public static final int
      PNEUMATIC_CONTROL_MODULE = 1,
      POWER_DISTRIBUTION_PANEL = 2,
      CANCODER_SHOULDER = 10,  
      CANCODER_ELBOW = 11,
      CANCODER_WRIST = 12,
      CANDLE = 15,
      CANIFIER = 16,
      INTAKE = 23,
      MAIN_ARM_LEADER = 24,
      MAIN_ARM_FOLLOWER = 26,
      SECONDARY_ARM_LEADER = 25,
      SECONDARY_ARM_FOLLOWER = 27,
      WRIST = 28,
      GROUND_INTAKE = 29,
      GROUND_WRIST = 30;
  }

  public static final class PDH {
    public static final int
      SWERVE_FL_DRIVE = 14, SWERVE_FL_TURN = 11,
      SWERVE_FR_DRIVE = 19, SWERVE_FR_TURN = 15,
      SWERVE_BR_DRIVE = 18, SWERVE_BR_TURN = 16,
      SWERVE_BL_DRIVE = 10, SWERVE_BL_TURN = 13,
      ARM_MAIN_A = 0, ARM_MAIN_B = 1,
      ARM_SECONDARY_A = 8, ARM_SECONDARY_B = 9,
      INTAKE = 3,
      RADIO = 21, RIO = 20,
      MINI_POWER_MODULE = 5;
  }

  public static final class SOLENOID {
    public static PneumaticsModuleType TYPE = PneumaticsModuleType.REVPH;

    public static final int J_HOOK_UP = 0, J_HOOK_DOWN = 1;
    public static final int WINGS_IN = 2;
    public static final int WINGS_OUT = 3;
  }

  public static final class TUNING {
    public static final boolean
      INFRASTRUCTURE = false,
      ARM_MAIN = false,
      ARM_SECONDARY = false,
      WINGS = false,
      J_HOOK = false,
      SWERVE = false,
      WRIST = false,
      VISION = false,
      CHARGE_STATION = false;
  }

  public static final class ARMS {
    public static final class TRIM {
      public static double
        VOLTS_PER_STICK_MAIN = 0.2,
        VOLTS_PER_STICK_SECONDARY = 0.3,
        DEGREES_PER_STICK_MAIN = 12.0,
        DEGREES_PER_STICK_SECONDARY = 9.0,
        INCHES_PER_STICK_X = 40.0,
        INCHES_PER_STICK_Z = 40.0;
    }

    public static final class MAIN {
      public static final double
        ON_TARGET_DEGREES = 2.5,
        ON_TARGET_DEGREES_PER_SECOND = 5.0;

      public static final double 
        GEAR_RATIO = (84.0 / 9.0) * (84.0 / 16.0) * (44.0 / 12.0),
        DEGREES_MIN = 0.0,
        DEGREES_MAX = 104.0,
        DEGREES_HOME = 103.0,
        GRAVITY_FF_PERCENT_VOLTAGE = 0.07,
        LENGTH = 40.0; //Inches

      public static final EncoderMath ENCODER_MATH = new EncoderMath(
        EncoderMath.TICKS_PER_ROTATION_CANCODER,
        EncoderMath.UNITS_PER_ROTATION_DEGREES,
        1.0 //no gearing between encoder and output mechanism
      );

      public static final double MAX_VELOCITY_NATIVE = 250.0;
      public static final MotionMagicParams MOTION_MAGIC = new MotionMagicParams(
        new PID(0.125, 0.0, 25, 1.0 / (MAX_VELOCITY_NATIVE / 1023.00)),
        MAX_VELOCITY_NATIVE * 0.3,
        1.0 / 2.5,
        0
      );

      // public static double MAX_VELOCITY_NATIVE = 222.63; //OG 20000 / FALCON_TICS_PER_ROATION / GEAR_RATIO * CANCODER_TICS_PER_ROTATION * CANCODER_GEAR_RATIO (195 deg/sec)
      // public static double MOTION_CRUISE_VELOCITY = 40 * (360.0 / 4096.0 / .1); //OG 4000: Desired deg/sec * degrees per roation / (CANCODER_TICS_PER_ROTATION * CANCODER_GEAR_RATIO) / 100 ms (.1)
      // public static double MOTION_ACCELERATION = MOTION_CRUISE_VELOCITY / .2; //OG MCV * 5: Desired Rise/Fall Time deg/sec^2 (Desired deg/sec / .1 ) / rise time * (degrees per roation / tics per rotaion) = MCV / rise time(s) 
      // public static PID PID = new PID(0.5, 0.0, 4.0, 1023.0 / MAX_VELOCITY_NATIVE);
      // public static int S_CURVE_STRENGTH = 2;

      public static final HardwareLimitParams LIMITS_HARDWARE = new HardwareLimitParams(true, true);
      public static final SoftwareLimitParams LIMITS_SOFTWARE = new SoftwareLimitParams(
        true, true,
        ENCODER_MATH.mechanismToEncoder(0.0),
        ENCODER_MATH.mechanismToEncoder(104.0)
      );
      
      public static final TalonFXParams TALONFX = new TalonFXParams(
        Constants.CAN.MAIN_ARM_LEADER,
        Constants.CAN.BUS_NAME,
        TalonFXInvertType.CounterClockwise,
        Constants.CAN.MAIN_ARM_FOLLOWER
      );
      public static final CANCoderParams CANCODER = new CANCoderParams(
        Constants.CAN.CANCODER_SHOULDER,
        Constants.CAN.BUS_NAME,
        Servo.configureCANCoder(AbsoluteSensorRange.Signed_PlusMinus180, true, SensorInitializationStrategy.BootToAbsolutePosition, -21.578516)
      );
    }

    public static final class SECONDARY {
      public static final double
        ON_TARGET_DEGREES = 2.5,  // TODO try 1.5 * MAIN
        ON_TARGET_DEGREES_PER_SECOND = 5.0;

      public static final double
        GEAR_RATIO = (84.0 / 9.0) * (84.0 / 16.0) * (42.0 / 12.0) * (44.0 / 24.0),
        DEGREES_MIN = 16.0,
        DEGREES_MAX = 138.0,
        DEGREES_HOME = 17.0,
        GRAVITY_OFFSET_FF = .02, //Percent Voltage
        LENGTH = 26.5; //Inches

      public static final EncoderMath ENCODER_MATH = new EncoderMath(
        EncoderMath.TICKS_PER_ROTATION_CANCODER,
        EncoderMath.UNITS_PER_ROTATION_DEGREES,
        1.0 //no gearing between encoder and output mechanism
      );

      public static final double MAX_VELOCITY_NATIVE = 165.0;
      public static final MotionMagicParams MOTION_MAGIC = new MotionMagicParams(
        new PID(3.0, 0.0, 40.0, (1023.0 / MAX_VELOCITY_NATIVE) * 0.65), // F*.65 to not overshoot
        MAX_VELOCITY_NATIVE * 0.6,
        1.0 / 2.5,
        0
      );

      // public static double MAX_VELOCITY_NATIVE = 127.2; //OG 20000 / FALCON_TICS_PER_ROATION / GEAR_RATIO * CANCODER_TICS_PER_ROTATION * CANCODER_GEAR_RATIO (111 deg/sec)
      // public static double MOTION_CRUISE_VELOCITY = 67 * (360.0 / 4096.0 / .1); // OG 12000: Desired deg/sec * degrees per roation / (CANCODER_TICS_PER_ROTATION * CANCODER_GEAR_RATIO) / 100 ms
      // public static double MOTION_ACCELERATION = MOTION_CRUISE_VELOCITY / .3; //OG MCV * : Desired Rise/Fall Time deg/sec^2 (Desired deg/sec / .1 ) / rise time * (degrees per roation / tics per rotaion) = MCV / rise time(s) 
      // public static PID PID = new PID(0.3, 0.0, 4.0, 1023.0 / MAX_VELOCITY_NATIVE);
      // public static int S_CURVE_STRENGTH = 0;

      public static final HardwareLimitParams LIMITS_HARDWARE = new HardwareLimitParams(true, true);
      public static final SoftwareLimitParams LIMITS_SOFTWARE = new SoftwareLimitParams(
        true, true,
        ENCODER_MATH.mechanismToEncoder(DEGREES_MIN),
        ENCODER_MATH.mechanismToEncoder(DEGREES_MAX)
      );
      
      public static final TalonFXParams TALONFX = new TalonFXParams(
        Constants.CAN.SECONDARY_ARM_LEADER,
        Constants.CAN.BUS_NAME,
        TalonFXInvertType.Clockwise,
        Constants.CAN.SECONDARY_ARM_FOLLOWER
      );
      public static final CANCoderParams CANCODER = new CANCoderParams(
        Constants.CAN.CANCODER_ELBOW,
        Constants.CAN.BUS_NAME,
        Servo.configureCANCoder(AbsoluteSensorRange.Signed_PlusMinus180, true, SensorInitializationStrategy.BootToAbsolutePosition, 22.675781)
      );
    }

    public static final ArmEnvelope ENVELOPE = new ArmEnvelope();
    static {
      // for different main arm angles, what's the range of motion for the secondary arm
      ENVELOPE.add(110.0, 16.7, 96.7);//  -59.4, 18.1 // Outside our limits
      ENVELOPE.add(100.0, 16.7, 96.7);//  -59.4, 18.1
      ENVELOPE.add(95.0, 19.4, 103.0);//  -65.0, 19.0
      ENVELOPE.add(90.0, 18.3, 119.9);//  -71.0, 30.0
      ENVELOPE.add(85.0, 18.9, 126.0);//  -76.0, 30.0
      ENVELOPE.add(80.0, 19.1, 118.2);//  -81.3, 17.2
      ENVELOPE.add(75.0, 19.4, 118.2);//  -85.9, 12.7
      ENVELOPE.add(70.0, 18.5, 126.0);//  -92.0, 15.2
      ENVELOPE.add(65.0, 18.9, 125.1);//  -97.0, 9.3
      ENVELOPE.add(60.0, 18.8, 128.8);//  -102.1, 2.6
      ENVELOPE.add(55.0, 18.8, 130.7);//  -106.7, -3.1
      ENVELOPE.add(50.0, 18.8, 132.4);//  -111.0, -5.2
      ENVELOPE.add(45.0, 45.4, 135.0);//  -89.5, 0.0
      ENVELOPE.add(40.0, 45.4, 137.0);//  -94.4, -3.0
      ENVELOPE.add(35.0, 45.2, 138.0);//  -100.0, -8.0
      // ENVELOPE.add(30.0, -69.0, -29.0);
    }

    public static final double ARM_SECONDARY_RAISED = (Constants.ARMS.SECONDARY.DEGREES_MAX + Constants.ARMS.SECONDARY.DEGREES_MIN) / 2.0;
  }

  public static final class DIAGNOSTICS {
    public static final DiagnosticState
      FAULT_NOT_ZEROED = new DiagnosticState(new LEDColor(0, 255, 0), true, true),
      NO_FAULT = new DiagnosticState(new LEDColor(255, 0, 0), false),
      MODE_CONE = new DiagnosticState(new LEDColor(255, 155, 0)),
      MODE_CONE_CAPTURED = new DiagnosticState(new LEDColor(255, 155, 0), false, true),
      MODE_CUBE = new DiagnosticState(new LEDColor(51, 0, 102)),
      MODE_CUBE_CAPTURED = new DiagnosticState(new LEDColor(51, 0, 102), false, true);
  }

  public static final class INTAKE {
    public static final double
      OPEN_LOOP_PERCENT = 0.4,
      CONE_IN_PERCENT = -OPEN_LOOP_PERCENT,
      CONE_OUT_PERCENT = OPEN_LOOP_PERCENT,
      CUBE_IN_PERCENT = OPEN_LOOP_PERCENT,
      CUBE_OUT_PERCENT = -OPEN_LOOP_PERCENT,
      RAMP_TO_FULL_SEC = .3; // time in seconds to 100%
  }

  public static final class STATE {
    public static final ArmState
      TRANSPORT = new ArmState(103.0, 17.0, 35.0),
      SCORE_CONE_TOP_PRE = new ArmState(95.0, 75.0, 78.0),
      SCORE_CONE_TOP = new ArmState(58.7, 128.2, 127.0),
      SCORE_CONE_TOP_END = new ArmState(58.7, 126.2, 133.2),

      SCORE_CONE_MIDDLE = new ArmState(80.2, 72.3, 107.5),

      SCORE_CUBE_TOP_PRE = new ArmState(95.0, 75.2, 100.0),
      SCORE_CUBE_TOP = new ArmState(53.0, 130.2, 120.0),

      SCORE_CUBE_MIDDLE = new ArmState(77.6, 68.0, 78.4),

      PICKUP_CUBE_GROUND_SLOW_DOWN = new ArmState(47, 50, 56.5),
      PICKUP_CUBE_GROUND = new ArmState(34.7, 55.6, 56.5),

      PICKUP_CONE_GROUND_SLOW_DOWN = new ArmState(55.0, 37.0, 96.5),//57.0, 53.4, 124.5),
      PICKUP_CONE_GROUND = new ArmState(55.0, 37.0, 96.5),//57.0, 53.4, 124.5),

      WALL_CUBE = new ArmState(90.0, 78.5, 107.5),
      WALL_CONE = new ArmState(90.0, 90.0, 158.5);
  }

  public static final class SUPERSTRUCTURE {
    public static boolean CAMERA_STREAM = true;
  }

  public static final class WRIST {
    public static final double
      VOLTS_PER_STICK = 1.0,
      DEGREES_PER_STICK = 16.0,
      ON_TARGET_DEGREES = 2.5,  // TODO since the wrist is very accurate, reduce this
      ON_TARGET_DEGREES_PER_SECOND = 5.0;

    public static final double
      GEAR_RATIO = 7.0 * 7.0 * 3.0 * (30.0 / 30.0),
      DEGREES_MIN = 33.0,
      DEGREES_MAX = 155.0,
      DEGREES_HOME = 35.0,
      GRAVITY_OFFSET_FF = .005, //Percent Voltage
      LENGTH = 14.0; //Inches

    public static final EncoderMath ENCODER_MATH = new EncoderMath(
      EncoderMath.TICKS_PER_ROTATION_CANCODER,
      EncoderMath.UNITS_PER_ROTATION_DEGREES,
      1.0 //no gearing between encoder and output mechanism
    );
    
    public static final double MAX_VELOCITY_NATIVE = 185.0; //OG 20000 / FALCON_TICS_PER_ROATION / GEAR_RATIO * CANCODER_TICS_PER_ROTATION * CANCODER_GEAR_RATIO (143 deg/sec)
    public static final MotionMagicParams MOTION_MAGIC = new MotionMagicParams(
      new PID(1.0, 0.0, 20.0, (1023.0 / MAX_VELOCITY_NATIVE) * 0.72), // F *.72 to not overshoot
      MAX_VELOCITY_NATIVE * 0.6, //115 * (360.0 / 4096.0 / .1); // OG MAX_VELOCITY_NATIVE * .8: Desired deg/sec * degrees per roation / (CANCODER_TICS_PER_ROTATION * CANCODER_GEAR_RATIO) / 100 ms
      1.0 / 3.0, //OG MCV * : Desired Rise/Fall Time deg/sec^2 (Desired deg/sec / .1 ) / rise time * (degrees per roation / tics per rotaion) = MCV / rise time(s) 
      0
    );

    public static final HardwareLimitParams LIMITS_HARDWARE = new HardwareLimitParams(true, true);
    public static final SoftwareLimitParams LIMITS_SOFTWARE = new SoftwareLimitParams(
      true, true,
      ENCODER_MATH.mechanismToEncoder(DEGREES_MIN),
      ENCODER_MATH.mechanismToEncoder(DEGREES_MAX)
    );

    public static final TalonFXParams TALONFX = new TalonFXParams(      
      Constants.CAN.WRIST,
      Constants.CAN.BUS_NAME,
      TalonFXInvertType.CounterClockwise
    );
    public static final CANCoderParams CANCODER = new CANCoderParams(
      Constants.CAN.CANCODER_WRIST,
      Constants.CAN.BUS_NAME,
      Servo.configureCANCoder(AbsoluteSensorRange.Signed_PlusMinus180, false, SensorInitializationStrategy.BootToAbsolutePosition, -100.0)
    );
  }

  public static final class SWERVE_DRIVE {
    public static final boolean DASHBOARD_ENABLED = true;

    public static final double
      SPEED_PERCENT_SLOW = .25,
      SPEED_PERCENT_NORMAL = .75,
      SPEED_PERCENT_FAST = 1.0,

      MAX_SPEED_MPS = 5.0, // 5.4 meters per second theoretical max for L3
      MAX_ACCEL_MPSS = 7.0, // from 0 to this mps in one second
      MAX_ROTATION_SPEED_DPS = 360, // degress in one second
      MAX_ROTATION_ACCEL_DPSS = 540, // from 0 to this degrees per second in one second
    
      TRANSPORT_SPEED_MPS = 1.5,

      AUTO_MAX_SPEED_MPS = 2.5,
      AUTO_MAX_ACCEL_MPS = 3.0,
      AUTO_MAX_ROTATION_SPEED_DPS = 290,
      AUTO_MAX_ROTATION_ACCEL_DPSS = 520,

      TIMEOUT_DISABLED_COAST = 5.0,

      // dimensions
      WHEEL_TRACK_WIDTH_X_METERS = Units.inchesToMeters(20.75),
      WHEEL_TRACK_WIDTH_Y_METERS = Units.inchesToMeters(28.05);

    // which CAN bus are the swerve components on
    public static final String CAN_BUS_NAME = "Swerve"; // "rio" is built-in, CANivore name is set via Phoenix Tuner

    // CAN identifiers for each module to get offsets, align all the bevel gears to the left of the robot,
    // use a straight edge to get them all precisely at aligned, then store the "Raw Enc Angle" of each below.
    // If correct, when wheels are all pointed forward, the "Abs Enc Angle" should be 0
    public static final SwerveModuleConfig[] MODULE_CONFIG = {
      new SwerveModuleConfig("FrontLeft",  14, 15, 16, Rotation2d.fromDegrees(218.5)),
      new SwerveModuleConfig("FrontRight", 17, 18, 19, Rotation2d.fromDegrees(295.4)),
      new SwerveModuleConfig("BackLeft",   11, 12, 13, Rotation2d.fromDegrees(86.2)),
      new SwerveModuleConfig("BackRight",  20, 21, 22, Rotation2d.fromDegrees(90.26))
    };

    private static double MAX_VELOCITY_NATIVE = 15000.0;
    public static PID DRIVE_PID = new PID(0.06, 0.0, 0.0, 1023.0 / MAX_VELOCITY_NATIVE);
    public static PID STEER_PID = new PID(0.2, 0.0, 0.1, 0.0);

    // gearbox configuration of the modules
    public static final SDSModuleConfig SDS_MODULE_CONFIG = SDSModuleConfig.SDSMK4i(SDSModuleConfig.Mk4i.L3, DRIVE_PID, STEER_PID);

    /** if true, when the track is idle it will try to resync the angle encoder */
    public static boolean STEER_IDLE_RESYNC_ENCODER = false;

    /** if true, the steering motor will try to directly use the remote cancoder as the sensor */
    public static boolean STEER_USE_REMOTE_SENSOR_FEATURE = false;

    public static final Translation2d[] MODULE_LOCATIONS = new Translation2d[] {
      new Translation2d( WHEEL_TRACK_WIDTH_X_METERS / 2.0,  WHEEL_TRACK_WIDTH_Y_METERS / 2.0), // frontLeft
      new Translation2d( WHEEL_TRACK_WIDTH_X_METERS / 2.0, -WHEEL_TRACK_WIDTH_Y_METERS / 2.0), // frontRight
      new Translation2d(-WHEEL_TRACK_WIDTH_X_METERS / 2.0,  WHEEL_TRACK_WIDTH_Y_METERS / 2.0), // backLeft
      new Translation2d(-WHEEL_TRACK_WIDTH_X_METERS / 2.0, -WHEEL_TRACK_WIDTH_Y_METERS / 2.0)  // backRight
    };

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      MODULE_LOCATIONS[0], MODULE_LOCATIONS[1], MODULE_LOCATIONS[2], MODULE_LOCATIONS[3]
    );

    public static final TrajectoryConfig TRAJECTORY_CONFIG =
      new TrajectoryConfig(AUTO_MAX_SPEED_MPS, AUTO_MAX_ACCEL_MPS)
        .setKinematics(KINEMATICS);
    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(AUTO_MAX_SPEED_MPS, AUTO_MAX_ACCEL_MPS);
    public static final double DRIVE_ARB_FEEDFORWARD = 0.035;

    // Drive Motor Characterization Values 
    // Divide SYSID values by 12 to convert from volts to percent output for CTRE
    public static final double
      DRIVE_KS = (0.30 / 12.0), //TODO: This must be tuned to specific robot
      DRIVE_KV = (1.40 / 12.0),
      DRIVE_KA = (0.25 / 12.0),

      P_X_CONTROLLER = 0.6,
      P_Y_CONTROLLER = 0.6,
      P_ANGLE_CONTROLLER = 5.0;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints ANGLE_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
        Math.toRadians(AUTO_MAX_ROTATION_SPEED_DPS), 
        Math.toRadians(AUTO_MAX_ROTATION_ACCEL_DPSS)
      );
  }
}