package frc.robot.drivetrain;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Conversions;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

public class Drivetrain extends SubsystemBase {

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark leftMotor = new Spark(RobotConstants.MotorIDs.driveMotorLeft);
  private final Spark rightMotor = new Spark(RobotConstants.MotorIDs.driveMotorRight);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

  // Set up the RomiGyro
  private final RomiGyro romiGyro = new RomiGyro();
  private final AnalogGyro m_gyro = new AnalogGyro(0); // Used for simulation

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  // Set up the differential drive controller
  private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotor, rightMotor);

  // Field Odometry and Simulation things
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
  public final Field2d fieldSim = new Field2d();
  private final DrivetrainSim driveSim;
  public final Advanced advanced;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightMotor.setInverted(true);

    // Use meters as unit for encoder distances
    leftEncoder.setDistancePerPulse((2 * Math.PI * DrivetrainConstants.kWheelRadius)
        / DrivetrainConstants.kCountsPerRevolution);
    rightEncoder.setDistancePerPulse((2 * Math.PI * DrivetrainConstants.kWheelRadius)
        / DrivetrainConstants.kCountsPerRevolution);
    resetOdometry();

    SmartDashboard.putData("Field", fieldSim); // This is how we can see the robot position on the field

    // SIMULATION ONLY THINGS
    driveSim = new DrivetrainSim(leftMotor, rightMotor, leftEncoder, rightEncoder, m_gyro);
    // ADAVANCED Drive things
    advanced = new Advanced(leftMotor, rightMotor, leftEncoder, rightEncoder, diffDrive);
  }

  // Arcade Drive = Throttle on one axis and steering on another axis
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void stop(){
    diffDrive.stopMotor();
  }

  /** Resets robot odometry. */
  public void resetOdometry() {
    leftEncoder.reset();
    rightEncoder.reset();
    resetGyro();
  }

  /** Update odometry - this should be run every robot loop. */
  public void periodic() {
    updateOdometry();
  }

  /** Update our simulation. This should be run every robot loop in simulation. */
  public void simulationPeriodic() {
    driveSim.simulationPeriodic();
  }

  /**
   * Update robot odometry. Odometry is the estimate of robot postion from
   * encoders and gyro
   */
  public void updateOdometry() {
    Rotation2d angle;
    if (Robot.romiConnected) {
      angle = Rotation2d.fromDegrees(romiGyro.getAngleZ());
    } else { // If we are just simulating the drivetrain
      angle = m_gyro.getRotation2d();
    }
    m_odometry.update(
        angle, leftEncoder.getDistance(), rightEncoder.getDistance());
    fieldSim.setRobotPose(m_odometry.getPoseMeters());
  }

  public void resetOdometry(Pose2d pose) {
    leftEncoder.reset();
    rightEncoder.reset();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(romiGyro.getAngleZ()));
  }

  // return odometry object
  public DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  /** Check the current robot pose. */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public double getLeftDistanceInch() {
    return Conversions.metersToInches(leftEncoder.getDistance());
  }

  public double getRightDistanceInch() {
    return Conversions.metersToInches(rightEncoder.getDistance());
  }

  public double getAverageDistanceInch() {
    return Conversions.metersToInches((getLeftDistanceInch() + getRightDistanceInch()) / 2.0);
  }

  /** The acceleration in the Z-axis in Gs. */
  public double getAccelZ() {
    return accelerometer.getZ();
  }

  /** Current angle of the Romi around the Z-axis in degrees. */
  public double getGyroAngleZ() {
    return romiGyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    romiGyro.reset();
    m_gyro.reset();
  }

}
