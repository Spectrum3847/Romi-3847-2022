// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.Robot;

/** Add your docs here. */
public class DrivetrainSim {

    private final Spark leftMotor;
    private final Spark rightMotor;
    
    // Simulation
    // Simulation classes help us simulate our robot
    private final AnalogGyroSim m_gyroSim;
    private final EncoderSim m_leftEncoderSim;
    private final EncoderSim m_rightEncoderSim;
    //private final Field2d m_fieldSim = new Field2d();
    private final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(
            DrivetrainConstants.kVLinear, DrivetrainConstants.kALinear, DrivetrainConstants.kVAngular,
            DrivetrainConstants.kAAngular, DrivetrainConstants.kTrackWidth);
    private final DifferentialDrivetrainSim m_drivetrainSimulator = new DifferentialDrivetrainSim(
            m_drivetrainSystem, DCMotor.getRomiBuiltIn(1), 1, DrivetrainConstants.kTrackWidth, DrivetrainConstants.kWheelRadius,
            null);

    public DrivetrainSim(Spark leftM, Spark rightM, Encoder leftEncoder, Encoder rightEncoder, AnalogGyro gyro) {
        leftMotor = leftM;
        rightMotor = rightM;
        m_gyroSim = new AnalogGyroSim(gyro);
        m_leftEncoderSim = new EncoderSim(leftEncoder);
        m_rightEncoderSim  = new EncoderSim(rightEncoder);
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
}
