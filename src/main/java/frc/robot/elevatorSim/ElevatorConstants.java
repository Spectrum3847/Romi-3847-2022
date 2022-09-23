// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevatorSim;

import edu.wpi.first.math.util.Units;
import frc.robot.RobotConstants;

/** Add your docs here. */
public class ElevatorConstants {
    public static final int kMotorPort = RobotConstants.MotorIDs.elevatorMotor;
    public static final int kEncoderAChannel = 8;
    public static final int kEncoderBChannel = 9;

    public static final double kElevatorKp = 5.0;
    public static final double kElevatorGearing = 10.0;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double kCarriageMass = 4.0; // kg

    public static final int kMinElevatorHeight = (int) Units.inchesToMeters(2);
    public static final int kMaxElevatorHeight = (int) Units.inchesToMeters(50);

    // distance per pulse = (distance per revolution) / (pulses per revolution)
    // = (Pi * D) / ppr
    public static final double kElevatorEncoderDistPerPulse =
            2.0 * Math.PI * ElevatorConstants.kElevatorDrumRadius / 4096;
}
