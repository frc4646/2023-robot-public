package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.team4646.SmartSubsystem;
import frc.team4646.Test;

public class Infrastructure extends SmartSubsystem {
  private class DataCache {
    public boolean enableCompressor = true;
    public double battery;
  }
  private class OutputCache {
    public boolean enableCompressor = true;
  }

  private final PneumaticHub pneumatics;
  private final PowerDistribution powerDist;

  private final DataCache cache = new DataCache();
  private final OutputCache outputs = new OutputCache();
  private int pressureMin = 90, pressureMax = 110;

  public Infrastructure() {
    pneumatics = new PneumaticHub(Constants.CAN.PNEUMATIC_CONTROL_MODULE);
    pneumatics.clearStickyFaults();
    // pneumatics.enableCompressorAnalog(pressureMin, pressureMax);
    pneumatics.enableCompressorDigital();

    powerDist = new PowerDistribution(Constants.CAN.POWER_DISTRIBUTION_PANEL, ModuleType.kRev);
    setArmsLEDs(false);

    // setCompressor(true);

    if (Constants.SUPERSTRUCTURE.CAMERA_STREAM) {
      new Thread(() -> {
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(416, 240);
        camera.setFPS(15);
      }).start();
    }
    
    if(Constants.TUNING.INFRASTRUCTURE) {
        createDashboard();
    }
  }

  @Override
  public void cacheSensors() {
    cache.battery = RobotController.getBatteryVoltage();
  }

  @Override
  public void updateHardware() {
    // pneumatics.enableCompressorAnalog(pressureMin, pressureMax);

    // updateCompressor();
  }
  
  @Override
  public void onEnable(boolean isAutonomous) {
    setArmsLEDs(false);
  }

  @Override
  public void onDisable() {
    setArmsLEDs(false);
  }

  @Override
  public void whileDisabled() {
    setArmsLEDs(true);
  }

  public void setArmsLEDs(boolean on) {
    powerDist.setSwitchableChannel(on);
  }

  
  // public void setCompressor(boolean enable) {
  //   outputs.enableCompressor = enable;
  // }

  // private void updateCompressor() {
  //   if (outputs.enableCompressor != cache.enableCompressor) {
  //     // cache.enableCompressor = outputs.enableCompressor;
  //     if(outputs.enableCompressor) {
  //       pneumatics.enableCompressorAnalog(pressureMin, pressureMax);
  //     } else {
  //       pneumatics.disableCompressor();
  //     }
  //   }
  // }

  public void createDashboard() {    
    ShuffleboardTab tab = Shuffleboard.getTab(getName());
    tab.addBoolean("Compressor", () -> cache.enableCompressor);
    // tab.addBoolean("Compressor Request", () -> outputs.enableCompressor);
    // tab.addDouble("High Pressure", () -> pneumatics.getPressure(0)).withPosition(0, 0);
    // tab.addDouble("Low Pressure", () -> pneumatics.getPressure(1)).withPosition(0, 1);

    tab.add(powerDist).withPosition(3, 0);
  }

  @Override
  public void runTests() {
    Test.add(this, "Compressor - Connected", !pneumatics.getStickyFaults().CanWarning && !pneumatics.getStickyFaults().CanBusOff);
    Test.add(this, "Compressor - Current Low", !pneumatics.getStickyFaults().CompressorOverCurrent);  // Max continuous 12V / 17A
    Test.add(this, "Compressor - Not Shorted", !pneumatics.getStickyFaults().CompressorOpen);
    Test.add(this, "Battery - Voltage", RobotController.getBatteryVoltage() > 12.5);
  }
}
