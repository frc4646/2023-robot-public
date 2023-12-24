// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

public class ArmState {
  public final ArmAngles arms;
  public final double wrist;

  public ArmState(double main, double secondary, double wrist) {
    this(new ArmAngles(main, secondary), wrist);
  }

  public ArmState(ArmAngles arms, double wrist) {
    this.arms = arms;
    this.wrist = wrist;
  }
}
