package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.configuration.OperatorControls;
import frc.team4646.Candle;
import frc.team4646.DiagnosticState;
import frc.team4646.LEDColor;
import frc.team4646.SmartSubsystem;

public class Diagnostics extends SmartSubsystem {
  public final LEDColor
    OFF = new LEDColor(0, 0, 0),
    RED = new LEDColor(255, 0, 0),
    BLUE = new LEDColor(0, 0, 255);

  private final Candle candle;
  private final OperatorControls operator = RobotContainer.operator;
  private LEDColor modeDefault = OFF, robotState = OFF;
  private boolean isCriticalIssuePresent = false;
  private boolean flashing;

  
  public Diagnostics() {
    candle = RobotContainer.CANDLE;
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  public void setState(DiagnosticState state) {
    robotState = state.color;
    flashing = state.flash;
    isCriticalIssuePresent = state.critical;
  }

  public void setStateOkay() {
    robotState = modeDefault;
    isCriticalIssuePresent = false;
  }

  private void setLEDs(LEDColor color, boolean flashing) {
    if(flashing) {
      // candle.set(0, 0, 0);
      candle.set(new StrobeAnimation(color.red, color.green, color.blue, 0, .05, Candle.LED_COUNT));
      // candle.set(new SingleFadeAnimation(color.red, color.green, color.blue, 0, .95, Candle.LED_COUNT));
    } else {
      // candle.set(new ColorFlowAnimation(color.red, color.green, color.blue, 0, .8, Candle.LED_COUNT, Direction.Forward));
      // candle.set(new LarsonAnimation(color.red, color.green, color.blue, 0, .8, Candle.LED_COUNT, BounceMode.Front, 7));
      candle.set(new SingleFadeAnimation(color.red, color.green, color.blue, 0, 0, Candle.LED_COUNT));
      // candle.set(color.red, color.green, color.blue);
      // candle.set(new StrobeAnimation(color.red, color.green, color.blue, 0, 0, Candle.LED_COUNT));
    }
  }

  private void setLEDs(Animation animation) {
    candle.set(animation);
  }

  private void setRumble(double percent) {
    operator.setRumble(true, percent);
    operator.setRumble(false, percent);
  }

  private LEDColor allianceColor() { return DriverStation.getAlliance() == Alliance.Red ? RED : BLUE; }

  @Override
  public void updateHardware() {
    setLEDs(robotState, flashing);
    // setRumble((DriverStation.isDisabled() && isCriticalIssuePresent) ? Constants.DIAGNOSTICS.RUMBLE_PERCENT : 0.0);
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    modeDefault = !isAutonomous ? OFF : allianceColor();
  }

  @Override
  public void onDisable() {
    modeDefault = allianceColor();
  }
}
