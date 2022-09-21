package frc.robot.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveTime extends CommandBase {
  private final double m_duration;
  private final double m_speed;
  private long m_startTime;

  /**
   * Creates a new DriveTime. This command will drive your robot for a desired speed and time.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param time How much time to drive in seconds
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveTime(double speed, double time) {
    m_speed = speed;
    m_duration = time * 1000;
    addRequirements(Robot.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
    Robot.drivetrain.arcadeDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.drivetrain.arcadeDrive(m_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - m_startTime) >= m_duration;
  }
}
