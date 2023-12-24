// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class ArmAngles {
  public final double mainDegrees, secondaryDegrees;

  public ArmAngles(Rotation2d mainDegrees, Rotation2d secondaryDegrees) {
    this(mainDegrees.getDegrees(), secondaryDegrees.getDegrees()); 
  }
  
  public ArmAngles(double mainDegrees, double secondaryDegrees) {
    this.mainDegrees = mainDegrees;
    this.secondaryDegrees = secondaryDegrees;
  }

  public ArmAngles() {
    this(0.0, 0.0);
  }

  public ArmAngles trim(double mainDegrees, double secondaryDegrees) {
    return new ArmAngles(this.mainDegrees + mainDegrees, this.secondaryDegrees + secondaryDegrees);
  }

  public static IntakeLocation toLocation(ArmAngles angles)
  {
    final double xInches = 
    Constants.ARMS.MAIN.LENGTH*Math.cos(Math.toRadians(angles.mainDegrees)) + Constants.ARMS.SECONDARY.LENGTH*Math.cos(Math.toRadians(angles.secondaryDegrees));

    final double zInches = 
    Constants.ARMS.MAIN.LENGTH*Math.sin(Math.toRadians(angles.mainDegrees)) + Constants.ARMS.SECONDARY.LENGTH*Math.sin(Math.toRadians(angles.secondaryDegrees));

    return new IntakeLocation(xInches, zInches);
  }
}
