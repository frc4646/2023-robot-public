// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.configuration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveSnapToAngle;
import frc.robot.commands.drive.DriveTeleopAroundCone;
import frc.robot.commands.group.GamepiecePickup;
import frc.robot.commands.vision.VisionBackSetPose;
import frc.robot.commands.vision.VisionBackSetPoseAndResetGyro;
import frc.team4646.Util;

public class DriverControlsSticks implements IDriverControls {
    private final double kJoystickDeadband = .2;
    private final int kJoystickPower = 1; // square or cube the input to get finer control at low input values
    private final CommandJoystick stickMove, stickRotate;

    private final int BUTTON_BOTTOM = 1, BUTTON_TOP = 2;

    public DriverControlsSticks() {
        stickMove = new CommandJoystick(0);
        stickRotate = new CommandJoystick(2);
        SmartDashboard.putData("Reset Gyro", new InstantCommand(RobotContainer.SWERVE::resetGyro, RobotContainer.SWERVE));
        SmartDashboard.putData("Reset Pose Apriltag", new VisionBackSetPose());

        // SmartDashboard.putData("Wrist 90", new WristPosition(90.0));
        // SmartDashboard.putData("Wrist 140", new WristPosition(140.0));

        stickRotate.button(BUTTON_TOP).whileTrue(new DriveTeleopAroundCone());
        stickRotate.button(BUTTON_BOTTOM).whileTrue(new DriveSnapToAngle());
        stickMove.button(BUTTON_BOTTOM).whileTrue(new GamepiecePickup());


        // if all 4 buttons are pressed
        stickMove.button(BUTTON_TOP)
            .and(stickMove.button(BUTTON_BOTTOM))
            .and(stickRotate.button(BUTTON_TOP))
            .and(stickRotate.button(BUTTON_BOTTOM))
            .onTrue(new VisionBackSetPoseAndResetGyro());
    }
 
    @Override
    public double getX() {
        double value = -Util.handleJoystick(stickMove.getY(), kJoystickDeadband, kJoystickPower);
        // SmartDashboard.putNumber("Joystick X", value);
        return value; 
    }

    @Override
    public double getY() {
        double value = -Util.handleJoystick(stickMove.getX(), kJoystickDeadband, kJoystickPower);
        // SmartDashboard.putNumber("Joystick Y", value);
        return value;  
    }

    @Override
    public double getRotation() {
        double value = -Util.handleJoystick(stickRotate.getX(), kJoystickDeadband, kJoystickPower); 
        // SmartDashboard.putNumber("Joystick Rotate", value);
        return value;  
    }

    @Override
    public boolean getFieldRelativeToggle() {
        return false;
    }

    @Override
    public boolean getSpeedBoost() {
    //    return stickMove.button(BUTTON_BOTTOM).getAsBoolean();
       return false;
    }

    @Override
    public boolean getSpeedSlow() {
        return stickMove.button(BUTTON_TOP).getAsBoolean() || stickMove.button(BUTTON_BOTTOM).getAsBoolean();
    }

    @Override
    public int getSnapAngle() {
        // get stick angle between 0 and 360
        if(stickRotate.getMagnitude() < kJoystickDeadband) {
          return -1;
        }
        double angle = (stickRotate.getDirectionDegrees() + 360) % 360.0;
        // SmartDashboard.putNumber("Joystick Angle", angle);

        // convert to robot heading
        if(angle < 45 || angle > 315) {
            return 0;
        } else if(angle < 135) {
            return 270;
        } else if(angle < 225) {
            return 180;
        } else {
            return 90;
        }
    }
}
