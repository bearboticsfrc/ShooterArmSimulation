// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
  public static final int kMotorPort = 0;
  public static final int kEncoderAChannel = 0;
  public static final int kEncoderBChannel = 1;
  public static final int kJoystickPort = 0;

  public static final String kArmPositionKey = "ArmPosition";
  public static final String kArmPKey = "ArmP";
  public static final String kArmIKey = "ArmI";
  public static final String kArmDKey = "ArmD";

  // The P gain for the PID controller that drives this arm.
  public static final double kDefaultArmKp = 0.8;
  public static final double kDefaultArmKi = 0.0;
  public static final double kDefaultArmKd = 0.05;
  public static final double kDefaultArmSetpointDegrees = 0.0;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses)
  public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  public static final double kArmReduction = 30;
  public static final double kArmMass = 9.0; // Kilograms
  public static final double kArmLength = Units.inchesToMeters(15.75);
  public static final double kMinAngleRads = Units.degreesToRadians(0);
  public static final double kMaxAngleRads = Units.degreesToRadians(255);


  public static final String kArmSKey = "FFS";
  public static final String kArmGKey = "FFG";
  public static final String kArmVKey = "FFV";
  public static final String kArmAKey = "FFA";
  public static final double kFeedForwardKs = 0.0;
  public static final double kFeedForwardKg = 0.115;
  public static final double kFeedForwardKv = 0.0;
  public static final double kFeedForwardKa = 0.0;

}
