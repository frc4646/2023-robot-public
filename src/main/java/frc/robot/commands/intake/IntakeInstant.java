// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeInstant extends InstantCommand {
    private final Intake subsystem = RobotContainer.INTAKE;
    private final double setpoint;

    /**
     * Runs the action instantly, useful for automated commands
     */
    public IntakeInstant(double percentVoltage) {
        addRequirements(subsystem);
        this.setpoint = percentVoltage;
    }

    public IntakeInstant() {
        this(0.0);
    }


    @Override
    public void initialize() {
        subsystem.setMotor(setpoint);
    }
}
