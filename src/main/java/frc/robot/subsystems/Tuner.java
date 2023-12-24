// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.auto.ChargingStationClimb;
import frc.team4646.SmartSubsystem;
import frc.team4646.Util;

public class Tuner extends SmartSubsystem {
  public static class TuneItem {
    public final String key;
    public final DoubleConsumer consumer;
    public final double supplier;
    public TuneItem(String key, DoubleConsumer consumer, double supplier) {
      this.key = key;
      this.consumer = consumer;
      this.supplier = supplier;
    }
  }

  private final List<Tune> tunes;
  private int grids;

  public Tuner() {
    tunes = new ArrayList<Tune>();
    grids = 0;
    add(
      "climb",
      new TuneItem("meter per second", ChargingStationClimb::setClimbSpeed, ChargingStationClimb.CLIMB_MPS),
      new TuneItem("degrees", ChargingStationClimb::setClimbDegrees, ChargingStationClimb.CLIMB_DEGREES),
      new TuneItem("timeout", ChargingStationClimb::setClimbTimeout, ChargingStationClimb.CLIMB_TIMEOUT)
    );
    add(
      "fall",
      new TuneItem("meter per second", ChargingStationClimb::setFallSpeed, ChargingStationClimb.FALL_MPS),
      new TuneItem("degrees", ChargingStationClimb::setFallDegrees, ChargingStationClimb.FALL_DEGREES),
      new TuneItem("timeout", ChargingStationClimb::setFallTimeout, ChargingStationClimb.FALL_TIMEOUT)
    );
    add(
      "reverse",
      new TuneItem("meter per second", ChargingStationClimb::setReverseSpeed, ChargingStationClimb.REVERSE_MPS),
      new TuneItem("timeout", ChargingStationClimb::setReverseTimeout, ChargingStationClimb.REVERSE_TIMEOUT)
    );

    Shuffleboard.getTab("Tuner").add(new ChargingStationClimb());
  }

  @Override
  public void cacheSensors() {
    tunes.forEach(Tune::update);
  }

  public void add(String layoutName, TuneItem... items) {
    ShuffleboardTab tab = Shuffleboard.getTab("Tuner");
    ShuffleboardLayout layout = tab.getLayout(layoutName, BuiltInLayouts.kGrid)
        .withSize(2, items.length)
        .withPosition(2 * grids++, 0)
        .withProperties(Map.of("Number of rows", items.length, "Number of columns", 1));
    int row = 0;
    for(int index = 0; index < items.length; index++) {
      TuneItem item = items[index];
      tunes.add(new Tune(layout, item.key, item.consumer, item.supplier, row++));
    }
  }

  public static class Tune {
    private final DoubleConsumer reference;
    private final GenericEntry widget;
    private final double valueDefault;
    private double valueLast;
    
    public Tune(ShuffleboardLayout layout, String key, DoubleConsumer consumer, double supplier, int row) {
      this.reference = consumer;
      this.valueDefault = supplier;
      this.valueLast = valueDefault;
      this.widget = layout.add(key, valueLast).withPosition(0, row).getEntry();
    }
  
    public void update() {
      final double valueCurrent = widget.getDouble(valueDefault);
      if (!Util.epsilonEquals(valueCurrent, valueLast)){
        reference.accept(valueCurrent);
      }
    }
  }
}
