package frc.robot.configuration;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.ModeCenter;
import frc.robot.commands.auto.ModeSide;
import frc.robot.util.GAMEPIECE_MODE;
import frc.robot.util.STARTING_LOCATION;

public class AutoModeSelector  {
  private final SendableChooser<String> modeSelector;
  private final Map<String, Optional<CommandBase>> modes;

  private final String defaultModeKey = "Do Nothing";
  private final CommandBase defaultMode = new WaitCommand(15.0);

  private Optional<CommandBase> currentMode;

  public AutoModeSelector() {
    modeSelector = new SendableChooser<>();
    modes = new HashMap<>();

    setDefault(defaultModeKey, defaultMode);

    // Add any auto commands here
    //add("Test Something", new AutoModeTestSomething());
    add("Center_________Balance", new ModeCenter(false, GAMEPIECE_MODE.NONE));
    add("Center_________Over_Balance_Cube", new ModeCenter(true, GAMEPIECE_MODE.CUBE));
    add("Center_________Over_Balance_Cone", new ModeCenter(true, GAMEPIECE_MODE.CONE));
    add("LoadingZone____Cube_Balance", new ModeSide(STARTING_LOCATION.LOADING_ZONE, GAMEPIECE_MODE.CUBE, true));
    add("LoadingZone____Cube", new ModeSide(STARTING_LOCATION.LOADING_ZONE, GAMEPIECE_MODE.CUBE, false));
    add("LoadingZone____Cone_Balance", new ModeSide(STARTING_LOCATION.LOADING_ZONE, GAMEPIECE_MODE.CONE, true));
    add("LoadingZone____Cone", new ModeSide(STARTING_LOCATION.LOADING_ZONE, GAMEPIECE_MODE.CONE, false));
    add("CableProtector_Cube_Balance", new ModeSide(STARTING_LOCATION.CABLE_PROTECTOR, GAMEPIECE_MODE.CUBE, true));
    add("CableProtector_Cube", new ModeSide(STARTING_LOCATION.CABLE_PROTECTOR, GAMEPIECE_MODE.CUBE, false));
    add("CableProtector_Cone_Balance", new ModeSide(STARTING_LOCATION.CABLE_PROTECTOR, GAMEPIECE_MODE.CONE, true));
    add("CableProtector_Cone", new ModeSide(STARTING_LOCATION.CABLE_PROTECTOR, GAMEPIECE_MODE.CONE, false));
    
    // push the chooser to the dashboard
    SmartDashboard.putData("Auto Mode", modeSelector);
  }

  /**
   * Add a new auto mode option
   */
  public void add(String name, CommandBase command) {
    modeSelector.addOption(name, name);
    modes.put(name, Optional.of(command));
  }

  /**
   * Set the default auto mode
   */
  public void setDefault(String name, CommandBase command) {
    modeSelector.setDefaultOption(name, name);
    modes.put(name, Optional.of(command));
  }

  /** 
   * Call to get the latest mode from the Dashboard
   */
  public void update() {
      String desiredModeKey = modeSelector.getSelected();

      if (desiredModeKey == null) {
        desiredModeKey = defaultModeKey;
      }

      Optional<CommandBase> desiredMode = modes.get(desiredModeKey);

      if (currentMode != desiredMode) {
          System.out.println("Auto: " + desiredMode);
          currentMode = desiredMode;
      }
  }

  /**
   * Reset the current mode to the default mode
   */
  public void reset() {
      currentMode = Optional.of(defaultMode);
  }

  /**
   * Get the currently selected Auto Mode
   */
  public Optional<CommandBase> getAutoMode() {
      return currentMode;
  }
}
