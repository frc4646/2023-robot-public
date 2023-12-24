package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.team4646.SmartSubsystem;

public class Vision extends SmartSubsystem {
  public final String DASH_NAME = "Vision";
  public enum LEDMode { PIPELINE, OFF, BLINK, ON }         // Order must match Limelight docs
  public enum CamMode { VISION_PROCESSOR, DRIVER_CAMERA }  // Order must match Limelight docs
  public enum VisionPipeline {VISIONTAPE, DETECTOR, APRILTAG}       // pipeline configuration, 0: traditional vision tape, 1: 3d apriltags in field space.
  public static class BasicTargetingData{
    public double area;         // Target visionTape.area (0% of image to 100% of image)
    public double xDegrees;     // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    public double yDegrees;     // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
    public String classID;
    public double horizontal;
    public double vertical;
    public String desiredDirection;
    public double xOffset, yOffset;
    public BasicTargetingData() {
      clear();
    }
    public void clear() {
      area = -1;
      xDegrees = -1;
      yDegrees = -1;
      horizontal = -1;
      vertical = -1;
      classID = "";
      desiredDirection = "";
      xOffset = -1;
      yOffset = -1;
    }
    
  }

  public static class AprilTagData{    
    public double [] botpose;                       // Size 6 array with Position(X, Y, Z) and Rotation(X, Y, Z)
    public double xPosition, yPosition, zRotation;  // used to store the botpose values for clearer use
    public int primaryID;                           // ID number of the primary tag used for the AprilTag calculations TODO: is there a way to see all the tags that are being used?
    public Pose2d pose;
    public double latency;
    public AprilTagData() {
      clear();
    }

    public void clear() {
      botpose = new double [] {};
      xPosition = -1;
      yPosition = -1;
      zRotation = -1;
      primaryID = -1;
      latency = -1;
      pose = new Pose2d();
    }
  }
  
  public static class DataCache {
    public VisionPipeline selectedPipeline;// which pipeline is selected (see Pipeline enum above)
    public boolean seesTarget;  // Whether the limelight has any valid targets
    public final AprilTagData aprilTag = new AprilTagData();
    public final BasicTargetingData visionTape = new BasicTargetingData();
    public final BasicTargetingData detector = new BasicTargetingData();
  }
  
  public static Field2d field = new Field2d();

  public Pose2d pose = new Pose2d();
  private final double fieldMap_yOffset = 4.0; 
  private final double fieldMap_xOffset = fieldMap_yOffset * (1654.0/802.0); 
  private final NetworkTable table;
  private final DataCache cache = new DataCache();

  public Vision() {
    this("limelight");
  }

  public Vision(String pipelineName)
  {
    table = NetworkTableInstance.getDefault().getTable(pipelineName);
    createDashboard(pipelineName);  }

  /** See https://docs.limelightvision.io/en/latest/networktables_api.html. */
  @Override
  public void cacheSensors() {
    cache.selectedPipeline = VisionPipeline.values()[(int) table.getEntry("getpipe").getDouble(0.0)];
    cache.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;

    if(cache.selectedPipeline == VisionPipeline.VISIONTAPE) {
      cache.aprilTag.clear();
      cache.detector.clear();
      cache.visionTape.xDegrees = table.getEntry("tx").getDouble(0.0);
      cache.visionTape.yDegrees = table.getEntry("ty").getDouble(0.0);
      cache.visionTape.area = table.getEntry("ta").getDouble(0.0);
      cache.visionTape.classID = "";
      cache.visionTape.horizontal = table.getEntry("thor").getDouble(0.0);
      cache.visionTape.vertical = table.getEntry("tvert").getDouble(0.0);
      cache.visionTape.xOffset = cache.visionTape.xDegrees + 25.1;
      cache.visionTape.yOffset = cache.visionTape.yDegrees + 7.5; 
      if(Math.abs(cache.visionTape.xDegrees + 25.1) < 1.0)
      {
        cache.visionTape.desiredDirection = "";
      }
      else if(cache.visionTape.xDegrees > -25.1)
      {
        cache.visionTape.desiredDirection = "back ";
      }
      else{
        cache.visionTape.desiredDirection = "forward ";
      }
      if(Math.abs(cache.visionTape.yDegrees + 7.5) < 1.0)
      {
        cache.visionTape.desiredDirection += "";
      }
      else if(cache.visionTape.xDegrees > -25.1)
      {
        cache.visionTape.desiredDirection += "left ";
      }
      else{
        cache.visionTape.desiredDirection += "right ";
      }
    }
    else if(cache.selectedPipeline == VisionPipeline.DETECTOR){
      cache.aprilTag.clear();
      cache.visionTape.clear();
      cache.detector.xDegrees = table.getEntry("tx").getDouble(0.0);
      cache.detector.yDegrees = table.getEntry("ty").getDouble(0.0);
      cache.detector.area = table.getEntry("ta").getDouble(0.0);
      cache.detector.classID = table.getEntry("tclass").getString("");
      cache.detector.horizontal = table.getEntry("thor").getDouble(0.0);
      cache.detector.vertical = table.getEntry("tvert").getDouble(0.0);
      cache.detector.xOffset = cache.detector.xDegrees + 24.1;
      cache.detector.yOffset = cache.detector.yDegrees + 7.5; 
}
    else {
      cache.visionTape.clear();
      cache.detector.clear();
      double [] botpose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
      cache.aprilTag.primaryID = (int) table.getEntry("tid").getDouble(0.0);
      if(botpose.length > 5) { 
            cache.aprilTag.botpose = botpose;
            cache.aprilTag.xPosition = botpose[0];
            cache.aprilTag.yPosition = botpose[1];
            cache.aprilTag.zRotation = botpose[5];
            cache.aprilTag.latency = botpose[6];
            
            Rotation2d heading = Rotation2d.fromDegrees(cache.aprilTag.zRotation);
            Translation2d position = new Translation2d(cache.aprilTag.xPosition, cache.aprilTag.yPosition);
            cache.aprilTag.pose = new Pose2d(position, heading);
            pose =  new Pose2d(new Translation2d(cache.aprilTag.xPosition + fieldMap_xOffset, cache.aprilTag.yPosition + fieldMap_yOffset), Rotation2d.fromDegrees(cache.aprilTag.zRotation));
            field.setRobotPose(pose);
        }
      }
    }
  

  @Override
  public void onEnable(boolean isAutonomous) {
    // setPipeline(VisionPipeline.APRILTAG);
  }

  @Override
  public void onDisable() {
    // setPipeline(VisionPipeline.APRILTAG); // new version of setLED (LED is off in apriltag pipeline)
  }

  public boolean foundAprilTag() {
    return cache.aprilTag.primaryID != -1;
  }
  public boolean foundVisionTape() {
    return cache.aprilTag.primaryID == -1 && cache.selectedPipeline == VisionPipeline.VISIONTAPE && cache.seesTarget;  //TODO This might be inverted
  }

  public boolean foundGamePiece() {
    return cache.detector.classID != "";
  }


  public void setLEDs(boolean on)
  {
    if(on)
    {
      table.getEntry("ledMode").setNumber(3);
    }
    else
    {
      table.getEntry("ledMode").setNumber(1);
    }

  }
  public void setPipeline(VisionPipeline pipeline) {
    if(pipeline != cache.selectedPipeline) {
      table.getEntry("pipeline").setNumber(pipeline.ordinal());
      cache.selectedPipeline = pipeline;
    }
  }

  /** swaps between vison tape and AprilTag pipelines,
   * @return the Pipeline it swaps to **/
  public VisionPipeline swapPipeline() {
    VisionPipeline otherPipeline;
    if(cache.selectedPipeline == VisionPipeline.APRILTAG) {
      otherPipeline = VisionPipeline.VISIONTAPE;
    }
    else {
      otherPipeline = VisionPipeline.APRILTAG;
    }
    setPipeline(otherPipeline);
    return otherPipeline;
  }

  /** @return the cached apriltag data, see AprilTagData for fields **/
  public AprilTagData getRobotPositionFromApriltag() {
    return cache.aprilTag;
  }

  /**  @return the cached visionTape data, see BasicTargetingData for fields **/  
  public BasicTargetingData getVisionTapePosition()
  {
    return cache.visionTape;
  }

  public BasicTargetingData getGamePiecePosition() {
    return cache.detector;
  }

  public boolean isTargetPresent() {
    return cache.seesTarget;
  }

  public VisionPipeline getCurrentPipeline()
  {
    return cache.selectedPipeline;
  }

  @Override
  public void runTests() {

  }

  public void createDashboard(String pipelineName) {
    ShuffleboardTab tab = Shuffleboard.getTab(pipelineName);

    if (Constants.TUNING.VISION) {
        // tab.add(field); 
        tab.addBoolean("Target", () -> cache.seesTarget).withPosition(0, 0);
        tab.addBoolean("lined up",      () -> Math.abs(cache.visionTape.xDegrees) < 1).withPosition(0, 1);
        tab.addNumber("Pipeline",      () -> cache.selectedPipeline.ordinal()).withPosition(0, 2);
        tab.addString("Direction", () -> cache.visionTape.desiredDirection).withPosition(0, 3);
        ShuffleboardLayout gridXY = tab.getLayout("Retrotape", BuiltInLayouts.kGrid).withPosition(1, 0).withSize(2, 2).withProperties(Map.of("Number of columns", 2, "Number of rows", 2));
        gridXY.addNumber("X",       () -> cache.visionTape.xDegrees).withPosition(0, 0);
        gridXY.addNumber("Y",       () -> cache.visionTape.yDegrees).withPosition(1, 0);
        gridXY.addNumber("xOffset",       () -> cache.visionTape.xOffset).withPosition(0, 1);
        gridXY.addNumber("yOffset",       () -> cache.visionTape.yOffset).withPosition(1, 1);
        ShuffleboardLayout gridAT = tab.getLayout("AprilTag", BuiltInLayouts.kGrid).withPosition(3, 0).withSize(2, 2).withProperties(Map.of("Number of columns", 2, "Number of rows", 2));
        gridAT.addNumber("AprilTag: X",     () -> cache.aprilTag.xPosition).withPosition(0, 0);
        gridAT.addNumber("AprilTag: Y",     () -> cache.aprilTag.yPosition).withPosition(1, 0);
        gridAT.addNumber("AprilTag: Z rotation",   () -> cache.aprilTag.zRotation).withPosition(0, 1);
        gridAT.addNumber("AprilTag: ID",           () -> cache.aprilTag.primaryID).withPosition(1, 1);
        ShuffleboardLayout grid = tab.getLayout("Detector", BuiltInLayouts.kGrid).withPosition(5, 0).withSize(2, 4).withProperties(Map.of("Number of columns", 2, "Number of rows", 4));
        grid.addString("Target",       () -> cache.detector.classID).withPosition(0, 0);
        grid.addNumber("Area",         () -> cache.detector.area).withPosition(1, 0);
        grid.addNumber("X",            () -> cache.detector.xDegrees).withPosition(0, 1);
        grid.addNumber("Y",            () -> cache.detector.yDegrees).withPosition(1, 1);
        grid.addNumber("Horizontal",   () -> cache.detector.horizontal).withPosition(0, 2);
        grid.addNumber("Vertical",     () -> cache.detector.vertical).withPosition(1, 2);
        grid.addNumber("Ratio",        () -> (cache.detector.vertical/cache.detector.horizontal)).withPosition(0, 3);
      }
  }
}
