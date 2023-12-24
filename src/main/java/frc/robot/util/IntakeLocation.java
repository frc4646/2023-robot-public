// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;

public class IntakeLocation {
  static final double len_main_in = Constants.ARMS.MAIN.LENGTH;
  static final double len_secondary_in = Constants.ARMS.SECONDARY.LENGTH;
  static final double envelop_max_in = len_main_in + len_secondary_in;
  static final double envelop_min_in = len_main_in - len_secondary_in;

  public final double xInches, zInches;

  /**
   * @param xInches intake distance in front of robot
   * @param zInches intake distance above robot
   */
  public IntakeLocation(double xInches, double zInches) {
    this.xInches = xInches;
    this.zInches = zInches;
  }

  public IntakeLocation() {
    this(0.0, 0.0);
  }

  /**
   * make sure that (x^2 + z^2) >= (main_length - secondary length)^2. And (x^2 + z^2) <= (main length + secondary length)^2
   * @param xInches intake length from origin in front of robot
   * @param zInches intake length from origin above robot
   * @return fancier math wont explode
   */
  public static boolean isLocationSafe(double xInches, double zInches) {
    final double len_intake_in_sq = xInches * xInches + zInches * zInches; 
    return len_intake_in_sq >= envelop_min_in * envelop_min_in && len_intake_in_sq <= envelop_max_in * envelop_max_in;    
  }

  public static boolean isLocationLegal(ArmAngles armsWanted, double wristWanted)
  {
    //height must be less than 6' 6" (78") from floor, must not reach more than 48" from frame perimiter
    double ArmZOffset = 7; //7 inches from the floor
    double ArmXOffset = -21; //inches from the front frame perimiter

    
    final double xInches = 
    Constants.ARMS.MAIN.LENGTH*Math.cos(Math.toRadians(armsWanted.mainDegrees)) + 
    Constants.ARMS.SECONDARY.LENGTH*Math.cos(Math.toRadians(armsWanted.secondaryDegrees)) +
    Constants.WRIST.LENGTH*Math.cos(Math.toRadians(wristWanted))+ 
    ArmXOffset;

    final double zInches = 
    Constants.ARMS.MAIN.LENGTH*Math.sin(Math.toRadians(armsWanted.mainDegrees)) + 
    Constants.ARMS.SECONDARY.LENGTH*Math.sin(Math.toRadians(armsWanted.secondaryDegrees)) +
    Constants.WRIST.LENGTH*Math.sin(Math.toRadians(wristWanted)) +
    ArmZOffset;

    if(xInches > 47.9 || zInches > 77.9)
      return false;
    else
      return true;
  }

  /** fancier math */
  public static ArmAngles toPosition(IntakeLocation intakeLocation) {
    if (!isLocationSafe(intakeLocation.xInches, intakeLocation.zInches)) {
      new PrintCommand("Unsafe fancy math").schedule();
      return Constants.STATE.TRANSPORT.arms;
    }

    final double L3 = Math.sqrt(intakeLocation.xInches * intakeLocation.xInches + intakeLocation.zInches * intakeLocation.zInches);

    //All angles are in radians
    double theta_1, theta_1_1, theta_1_2, theta_2_1, theta_2;
    
    theta_1_1 = Math.atan2(intakeLocation.zInches, intakeLocation.xInches);
    theta_1_2 = Math.acos((len_main_in * len_main_in + L3 * L3 - len_secondary_in * len_secondary_in ) / (2 * len_main_in * L3));
    theta_1 = theta_1_1 + theta_1_2;
    
    theta_2_1 = Math.acos((len_main_in * len_main_in + len_secondary_in * len_secondary_in - L3 * L3) / (2 * len_main_in * len_secondary_in));
    theta_2 = Math.PI + theta_1 + theta_2_1;

    //Get values in the +/- 180 range
    if(theta_1 >= Math.PI) theta_1 = theta_1 - 2*Math.PI;
    if(theta_2 >= Math.PI) theta_2 = theta_2 - 2*Math.PI;
    if(theta_1 < -1 * Math.PI) theta_1 = theta_1 + 2*Math.PI;
    if(theta_2 < -1 * Math.PI) theta_2 = theta_2 + 2*Math.PI;

    final double angleShoulder = Math.toDegrees(theta_1);
    final double angleElbow = Math.toDegrees(theta_2);
    return new ArmAngles(angleShoulder, angleElbow);
  }
}
