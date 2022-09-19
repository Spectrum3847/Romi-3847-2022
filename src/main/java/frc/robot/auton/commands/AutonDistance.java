// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.drivetrain.commands.DriveDistance;
import frc.robot.drivetrain.commands.TurnDegrees;

public class AutonDistance extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonDistance() {
    addCommands(
        new DriveDistance(-0.5, 10, Robot.drivetrain),
        new TurnDegrees(-0.5, 180, Robot.drivetrain),
        new DriveDistance(-0.5, 10, Robot.drivetrain),
        new TurnDegrees(0.5, 180, Robot.drivetrain));
  }
}
