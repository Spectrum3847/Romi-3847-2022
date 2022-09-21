// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

/** Add your docs here. */
public class AutonCommands {

    public static Command intializePathFollowing(Trajectory path){
        return new SequentialCommandGroup(
            AutonCommands.resetOdometry(path) //reset odometry to the initial position
        );
    }

    public static Command resetOdometry(Trajectory path){
        return new InstantCommand(() -> Robot.drivetrain.odometry.resetOdometry(path.getInitialPose()));
    }
}
