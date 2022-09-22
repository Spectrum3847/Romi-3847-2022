package frc.robot.drivetrain;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class Drivetrain extends SubsystemBase {

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  public final Spark leftMotor = new Spark(RobotConstants.MotorIDs.driveMotorLeft);
  public final Spark rightMotor = new Spark(RobotConstants.MotorIDs.driveMotorRight);

  // Set up the differential drive controller
  public final DifferentialDrive diffDrive = new DifferentialDrive(leftMotor, rightMotor);

  public final Odometry odometry;
  public final DrivetrainSim driveSim;
  public final Advanced advanced;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightMotor.setInverted(true);

    odometry = new Odometry(this);
    // SIMULATION ONLY THINGS
    driveSim = new DrivetrainSim(this);
    // ADAVANCED Drive things
    advanced = new Advanced(this);
  }

  // Arcade Drive = Throttle on one axis and steering on another axis
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void stop(){
    diffDrive.stopMotor();
  }

  /** Update odometry - this should be run every robot loop. */
  public void periodic() {
    odometry.updateOdometry();
  }

  /** Update our simulation. This should be run every robot loop in simulation. */
  public void simulationPeriodic() {
    driveSim.simulationPeriodic();
  }
}
