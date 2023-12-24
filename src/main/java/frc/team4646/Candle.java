package frc.team4646;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleFaults;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import frc.robot.Constants;

public class Candle extends SmartSubsystem {
  public static final int LED_COUNT = 32;
  public static Animation toColor(int red, int green, int blue, double percentFade) { return new SingleFadeAnimation(red, green, blue, 0, percentFade, LED_COUNT); }
  public static double BRIGHT = 1.0, DIM = 0.2;
  public static double FAST = 1.0, SLOW = 0.2, STATIC = 0.0;

  public static final Animation 
    OFF = toColor(0, 0, 0, 0.0),
    BLUE_FADE = toColor(0, 0, 255, 1.0),  // TODO tune
    BLUE_SOLID = toColor(0, 0, 255, 0.5),
    RED_FADE = toColor(255, 0, 0, 1.0),  // TODO tune
    RED_SOLID = toColor(255, 0, 0, 0.5);

  public static final LEDColor
    COLOR_OFF = new LEDColor(0, 0, 0);

  private final CANdle candle;
  
  public Candle(int CAN_ID) {
    CANdleConfiguration config = new CANdleConfiguration();
    config.statusLedOffWhenActive = true;
    config.disableWhenLOS = false;
    config.stripType = LEDStripType.GRB;
    config.brightnessScalar = 1.0;
    config.vBatOutputMode = VBatOutputMode.Off;
        
    candle = new CANdle(CAN_ID);
    candle.configAllSettings(config, Constants.CAN_TIMEOUT);
  }

  public void set(int red, int green, int blue) {
    candle.setLEDs(red, green, blue);
  }

  public void set(Animation color) {
    candle.animate(color);
  }

  @Override
  public void runTests() {
    CANdleFaults faults = new CANdleFaults();
    candle.getFaults(faults);

    Test.add(this, "Candle - Fault", !faults.hasAnyFault());

    System.out.println(String.format("Frame 1: %d", candle.getStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General)));
    System.out.println(String.format("Frame 2: %d", candle.getStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_2_Startup)));
    System.out.println(String.format("Frame 3: %d", candle.getStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_3_FirmwareApiStatus)));
    System.out.println(String.format("Frame 4: %d", candle.getStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_4_ControlTelem)));
    System.out.println(String.format("Frame 5: %d", candle.getStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_5_PixelPulseTrain)));
    System.out.println(String.format("Frame 6: %d", candle.getStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_6_BottomPixels)));
    System.out.println(String.format("Frame 7: %d", candle.getStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_7_TopPixels)));

    Animation COLOR_FLOW = new ColorFlowAnimation(255, 0, 0, 0, SLOW, LED_COUNT, Direction.Forward);
    Animation FIRE = new FireAnimation(BRIGHT, SLOW, LED_COUNT, 0.8, 0.2);
    Animation LARSON = new LarsonAnimation(255, 0, 0, 0, SLOW, LED_COUNT, BounceMode.Front, 4);
    Animation RAINBOW = new RainbowAnimation(BRIGHT, SLOW, LED_COUNT);
    Animation RGB_FADE = new RgbFadeAnimation(BRIGHT, FAST, LED_COUNT);
    Animation SINGLE_FADE = new SingleFadeAnimation(255, 0, 0, 0, STATIC, LED_COUNT);
    Animation STROBE = new StrobeAnimation(255, 0, 0, 0, STATIC, LED_COUNT);
    Animation TWINKLE = new TwinkleAnimation(255, 0, 0, 0, STATIC, LED_COUNT, TwinklePercent.Percent100);
    Animation TWINKLE_OFF = new TwinkleOffAnimation(255, 0, 0, 0, STATIC, LED_COUNT, TwinkleOffPercent.Percent100);
    List<Animation> ALL_ANIMATIONS = Arrays.asList(COLOR_FLOW, FIRE, LARSON, RAINBOW, RGB_FADE, SINGLE_FADE, STROBE, TWINKLE, TWINKLE_OFF);
  
    // for(Animation animation : ALL_ANIMATIONS) {
    //   Test.add(animation.getClass().getSimpleName(), true);
    //   candle.animate(animation);
    //   Timer.delay(5.0);
    // }
  }
}
