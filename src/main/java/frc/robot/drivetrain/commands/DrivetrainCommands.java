// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;

/** Add your docs here. */
public class DrivetrainCommands {
    
    public static void setupDefaultCommand(){
        Robot.drivetrain.setDefaultCommand(
            new RunCommand(() -> Robot.drivetrain.arcadeDrive(Robot.driverGamepad.getDriveThrottle(), 
                                Robot.driverGamepad.getDriveSteering()), 
                                Robot.drivetrain));
    }

    public static Command stop(){
        return new RunCommand(() -> Robot.drivetrain.stop(), Robot.drivetrain);
    }
}
