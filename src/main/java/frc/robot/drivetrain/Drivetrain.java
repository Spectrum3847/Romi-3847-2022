package frc.robot.drivetrain;

import org.opencv.core.RotatedRect;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Drivetrain extends SubsystemBase {

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark leftMotor = new Spark(0);
  private final Spark rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the RomiGyro
  private final RomiGyro romiGyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  // Set up the differential drive controller
  private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotor, rightMotor);

  private final AnalogGyro m_gyro = new AnalogGyro(0); // Used for simulation

  //PID Controllers
  private final PIDController m_leftPIDController = new PIDController(8.5, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(8.5, 0, 0);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
      DrivetrainConstants.kTrackWidth);
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  // Simulation
  // Simulation classes help us simulate our robot
  private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
  private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
  private final Field2d m_fieldSim = new Field2d();
  private final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(
      DrivetrainConstants.kVLinear, DrivetrainConstants.kALinear, DrivetrainConstants.kVAngular,
      DrivetrainConstants.kAAngular);
  private final DifferentialDrivetrainSim m_drivetrainSimulator = new DifferentialDrivetrainSim(
      m_drivetrainSystem, DCMotor.getCIM(2), 8, DrivetrainConstants.kTrackWidth, DrivetrainConstants.kWheelRadius,
      null);
          
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightMotor.setInverted(true);

    // Use meters as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((2 * Math.PI * DrivetrainConstants.kWheelRadius)
        / DrivetrainConstants.kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((2 * Math.PI * DrivetrainConstants.kWheelRadius)
        / DrivetrainConstants.kCountsPerRevolution);
    resetEncoders();

    SmartDashboard.putData("Field", m_fieldSim);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

    /** Update robot odometry. */
    public void updateOdometry() {
      Rotation2d angle;
      if (Robot.romiConnected){
        angle = Rotation2d.fromDegrees(romiGyro.getAngleZ());
      } else {
        angle = m_gyro.getRotation2d();
      }
      m_odometry.update(
          angle, m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    }
  
    /** Resets robot odometry. */
    public void resetOdometry(Pose2d pose) {
      m_leftEncoder.reset();
      m_rightEncoder.reset();
      m_drivetrainSimulator.setPose(pose);
      m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }
  
    /** Check the current robot pose. */
    public Pose2d getPose() {
      return m_odometry.getPoseMeters();
    }
  /** Update our simulation. This should be run every robot loop in simulation. */
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    if (!Robot.romiConnected) { // Only update these if romi isn't connected
      m_drivetrainSimulator.setInputs(
          leftMotor.get() * RobotController.getInputVoltage(),
          rightMotor.get() * RobotController.getInputVoltage());
      m_drivetrainSimulator.update(0.02);

      m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
      m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
      m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
      m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
      m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
    }
  }

  /** Update odometry - this should be run every robot loop. */
  public void periodic() {
    updateOdometry();
    m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return romiGyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return romiGyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return romiGyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    romiGyro.reset();
  }

}
