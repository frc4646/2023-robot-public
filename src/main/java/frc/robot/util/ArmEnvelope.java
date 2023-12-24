// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

/** Calculates the safe range of the secondary arm based on the position of the main arm */
public class ArmEnvelope {
    private final InterpolatingTreeMap<Double, Double> mapMinSecondary = new InterpolatingTreeMap<>();
    private final InterpolatingTreeMap<Double, Double> mapMaxSecondary = new InterpolatingTreeMap<>();
    private double minDegreesMain = Double.MAX_VALUE;
    private double maxDegreesMain = Double.MIN_VALUE;
    private double minDegreesSecondary = Double.MAX_VALUE;
    private double maxDegreesSecondary = Double.MIN_VALUE;
  
    public void add(double degreesMain, double minDegreesSecondary, double maxDegreesSecondary) {
      mapMinSecondary.put(degreesMain, minDegreesSecondary);
      mapMaxSecondary.put(degreesMain, maxDegreesSecondary);
      updateRange(degreesMain, minDegreesSecondary, maxDegreesSecondary);
    }
  
    public double getEnvelopMinDegreesSecondary(double degreesMain) { return mapMinSecondary.get(degreesMain); }
    public double getEnvelopMaxDegreesSecondary(double degreesMain) { return mapMaxSecondary.get(degreesMain); }

    public double getMinDegreesMain() { return minDegreesMain; }
    public double getMaxDegreesMain() { return maxDegreesMain; }
    public double getMinDegreesSecondary() { return minDegreesSecondary; }
    public double getMaxDegreesSecondary() { return maxDegreesSecondary; }

    public double getDefaultDegreesMain() { return (getMinDegreesMain() + getMaxDegreesMain()) / 2.0; }
    public double getDefaultMinDegreesSecondary() { return getEnvelopMinDegreesSecondary(getDefaultDegreesMain()); }
    public double getDefaultMaxDegreesSecondary() { return getEnvelopMaxDegreesSecondary(getDefaultDegreesMain()); }
  
    private void updateRange(double degreesMain, double minDegreesSecondary, double maxDegreesSecondary) {
      if (degreesMain < this.minDegreesMain) {
        this.minDegreesMain = degreesMain;
      }
      if (degreesMain > this.maxDegreesMain) {
        this.maxDegreesMain = degreesMain;
      }
      if (minDegreesSecondary < this.minDegreesSecondary) {
        this.minDegreesSecondary = minDegreesSecondary;
      }
      if (maxDegreesSecondary > this.maxDegreesSecondary) {
        this.maxDegreesSecondary = maxDegreesSecondary;
      }
    }

    
    public void createDashboardGrid(String tab, String widget, int xPosition, Supplier<Rotation2d> funcMainPosition) {
      final ShuffleboardLayout grid = Shuffleboard.getTab(tab).getLayout(widget, BuiltInLayouts.kGrid);
      grid.withPosition(xPosition, 1);
      grid.withSize(4, 2);
      grid.withProperties(Map.of("Number of columns", 3, "Number of rows", 5));

      grid.addDouble("Max Main", () -> getMaxDegreesMain()).withPosition(0, 0);
      grid.addDouble("Min Main", () -> getMinDegreesMain()).withPosition(0, 1);
      grid.addDouble("Max Secondary", () -> getMaxDegreesSecondary()).withPosition(1, 1);
      grid.addDouble("Min Secondary", () -> getMinDegreesSecondary()).withPosition(1, 0);

      grid.addDouble("Default Max", () -> getDefaultMaxDegreesSecondary()).withPosition(2, 0);
      grid.addDouble("Default Min", () -> getDefaultMinDegreesSecondary()).withPosition(2, 1);

      grid.addDouble("Envelope Max", () -> getEnvelopMaxDegreesSecondary(funcMainPosition.get().getDegrees())).withPosition(3, 0);
      grid.addDouble("Envelope Min", () -> getEnvelopMinDegreesSecondary(funcMainPosition.get().getDegrees())).withPosition(3, 1);
    }
  }
  
