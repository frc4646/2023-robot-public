// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.group;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.arms.ArmPosition;
import frc.robot.commands.wrist.WristPosition;
import frc.robot.util.ArmState;

public class ArmStateNonblocking extends ParallelDeadlineGroup {
  public ArmStateNonblocking(ArmState state) {
    super(new ArmPosition(state.arms), new WristPosition(state.wrist));
  }
}
