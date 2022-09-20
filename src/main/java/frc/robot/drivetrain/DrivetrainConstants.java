// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

/** Add your docs here. */
public class DrivetrainConstants {
  public static final double kMaxSpeed = 0.6;
  public static final double kMaxAccel = 0.4;
  public static final double kMaxAngularSpeed = Math.PI/4;
  
  public static final double kCountsPerRevolution = 1440.0;
  public static final double kWheelRadius = 0.035; // 70mm diameter meters / 2 for radius
  public static final double kTrackWidth = 0.14;

  public static final double kPLeft = 13;
  public static final double kPRight = 13;
  
  public static final double ramseteB = 2.1;
  public static final double ramseteZeta = 0.8;

  public static final double kS = 0.38069;
  public static final double kVLinear = 9.5975;
  public static final double kALinear = 0.60273;
  public static final double kVAngular = 20;
  public static final double kAAngular = 1;


}
