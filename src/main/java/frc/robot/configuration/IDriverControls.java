// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.configuration;

public interface IDriverControls {
    /**
     * Get the x speed. We are inverting this because Xbox controllers
     * return negative values when we push forward.
     */
    public double getX();

    /**
     * Get the y speed or sideways/strafe speed. We are inverting this because
     * we want a positive value when we pull to the left. Xbox controllers
     * return positive values when you pull to the right by default.
     */
    public double getY();

    /**
     * Get the rate of angular rotation. We are inverting this because we want a
     * positive value when we pull to the left (remember, CCW is positive in
     * mathematics). Xbox controllers return positive values when you pull to
     * the right by default.
     */
    public double getRotation();

    public boolean getFieldRelativeToggle();

    public boolean getSpeedBoost();

    public boolean getSpeedSlow();

    public int getSnapAngle();
}
