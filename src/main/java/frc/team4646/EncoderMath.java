// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4646;

/** Converts between encoder units and human-friendly units for the end mechanism moved by the motor */
public class EncoderMath {
  public static final double TICKS_PER_ROTATION_FALCON = 2048.0;
  public static final double TICKS_PER_ROTATION_CANCODER = 4096.0;

  public static final double UNITS_PER_ROTATION_DEGREES = 360.0;
  
  private final double encoderUnitsPerMechansimRotation;
  private final double mechanismUnitsPerEncoderRotation;

  /**
   * @param encoderUnitsPerEncoderRotation Encoder units per rotation <i>of the encoder itself</i>
   * @param mechanismUnitsPerEncoderRotation End mechanism units per rotation <i>of the encoder itself</i>
   * @param gearRatio Gear ratio between the encoder and end mechanism moved by the motor
   */
  public EncoderMath(double encoderUnitsPerEncoderRotation, double mechanismUnitsPerEncoderRotation, double gearRatio) {
    this.encoderUnitsPerMechansimRotation = encoderUnitsPerEncoderRotation * gearRatio;
    this.mechanismUnitsPerEncoderRotation = mechanismUnitsPerEncoderRotation;
  }

  /**
   * Converts units
   * @param encoderUnits Units reported by the encoder
   * @return Units are in degrees (for rotational mechanisms) or inches (for linear mechanisms)
   */
  public double encoderToMechanism(double encoderUnits) {
    return encoderUnits * (mechanismUnitsPerEncoderRotation / encoderUnitsPerMechansimRotation);
  }

  /**
   * Converts units
   * @param mechanismUnits Units are in degrees (for rotational mechanisms) or inches (for linear mechanisms)
   * @return Units reported by the encoder
   */
  public double mechanismToEncoder(double mechanismUnits) {
    return mechanismUnits / (mechanismUnitsPerEncoderRotation / encoderUnitsPerMechansimRotation);
  }
}
