// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Robot;

public class FollowTrajectory extends RamseteCommand {

  Trajectory traj;

  /** Creates a new FollowPPTrajectory. */
  public FollowTrajectory(Trajectory trajectory) {
    super(trajectory, () -> Robot.drivetrain.odometry.getPose(), Robot.drivetrain.advanced.ramseteController,
    Robot.drivetrain.advanced.feedforward, Robot.drivetrain.advanced.kinematics, () -> Robot.drivetrain.odometry.getWheelSpeeds(),
    Robot.drivetrain.advanced.leftPid, Robot.drivetrain.advanced.rightPid, Robot.drivetrain.advanced::tankDriveVolts);
    traj = trajectory;
    this.addRequirements(Robot.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    //Robot.drivetrain.advanced.plotTrajectory(traj);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(isFinished());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
