package frc.team4646;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierFaults;
import com.ctre.phoenix.CANifierStatusFrame;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;

public class Canifier extends SmartSubsystem {
  private class DataCache {
    public LEDColor color = new LEDColor();
    public boolean isConeDetected, isCubeDetected;
  }
  private class OutputCache {
    public LEDColor color = new LEDColor();
  }

  private final CANifier canifier;
  private final DataCache cache = new DataCache();
  private final OutputCache outputs = new OutputCache();

  public Canifier(int deviceID) {
    canifier = new CANifier(deviceID);
    canifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 10, Constants.CAN_TIMEOUT);
    // canifier.setStatusFramePeriod(CANifierStatusFrame.Status_1_General, 50, Copnstants.CAN_TIMEOUT);
    // canifier.setStatusFramePeriod(CANifierStatusFrame.Status_3_PwmInputs0, 1000, Constants.CAN_TIMEOUT);
    // canifier.setStatusFramePeriod(CANifierStatusFrame.Status_4_PwmInputs1, 1000, Constants.CAN_TIMEOUT);
    // canifier.setStatusFramePeriod(CANifierStatusFrame.Status_5_PwmInputs2, 1000, Constants.CAN_TIMEOUT);
    // canifier.setStatusFramePeriod(CANifierStatusFrame.Status_6_PwmInputs3, 1000, Constants.CAN_TIMEOUT);

    Shuffleboard.getTab(Constants.COMP_DASH_NAME).addBoolean("ConeDetected", () -> cache.isConeDetected);
    Shuffleboard.getTab(Constants.COMP_DASH_NAME).addBoolean("CubeDetected", () -> cache.isCubeDetected);
  }

  @Override
  public void cacheSensors() {
    cache.isConeDetected = !canifier.getGeneralInput(CANifier.GeneralPin.QUAD_B);
    cache.isCubeDetected = !canifier.getGeneralInput(CANifier.GeneralPin.QUAD_A);
  }

  @Override
  public void updateHardware() {
    updateLEDs();
  }

  public void setLEDs(LEDColor color) { outputs.color = color; }
  public boolean isConeDetected() { return cache.isConeDetected; }
  public boolean isCubeDetected() { return cache.isCubeDetected; }

  private void updateLEDs() {
    if (!cache.color.isEqual(outputs.color)) {      
      cache.color = outputs.color;
      canifier.setLEDOutput(cache.color.red / 255.0, CANifier.LEDChannel.LEDChannelB);
      canifier.setLEDOutput(cache.color.green / 255.0, CANifier.LEDChannel.LEDChannelA);
      canifier.setLEDOutput(cache.color.blue / 255.0, CANifier.LEDChannel.LEDChannelC);
    }
  }

  @Override
  public void runTests() {
    CANifierFaults faults = new CANifierFaults();
    canifier.getFaults(faults);

    Test.checkFirmware(this, canifier);
    // Test.add(this, "Turret Home", cache.isConeDetected, isCubeDetected);
    Test.add(this, String.format("Faults: 0x%X", faults.toBitfield()), faults.hasAnyFault());
  }
}
