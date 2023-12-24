// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.group;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arms.ArmPosition;
import frc.robot.commands.wrist.WristPosition;

public class TransportMode extends ParallelCommandGroup {
  public TransportMode() {
    addCommands(
      new ArmPosition(Constants.STATE.TRANSPORT.arms).alongWith( 
        new WristPosition(Constants.STATE.TRANSPORT.wrist)
        // new WristGoHome()
      )
    );
  }
}
