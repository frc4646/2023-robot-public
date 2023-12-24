// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.diagnostics;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Robotstate;
import frc.robot.util.GAMEPIECE_MODE;

public class ToggleGamepieceMode extends InstantCommand {
    private final Robotstate subsystem = RobotContainer.STATE;

    @Override
    public void initialize() {
        if (subsystem.isConeMode()) {
            subsystem.setMode(GAMEPIECE_MODE.CUBE);
        } else {
            subsystem.setMode(GAMEPIECE_MODE.CONE);
        }
    }
}
