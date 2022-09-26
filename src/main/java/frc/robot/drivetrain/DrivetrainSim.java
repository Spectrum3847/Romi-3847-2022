// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.Robot;

/** Add your docs here. */
public class DrivetrainSim {

    private final Drivetrain dt;

    // Simulation
    // Simulation classes help us simulate our robot
    private final AnalogGyroSim m_gyroSim;
    private final EncoderSim m_leftEncoderSim;
    private final EncoderSim m_rightEncoderSim;
    // private final Field2d m_fieldSim = new Field2d();
    private final LinearSystem<N2, N2, N2> m_drivetrainSystem;
    private final DifferentialDrivetrainSim m_drivetrainSimulator;

    public DrivetrainSim(Drivetrain dt) {
        this.dt = dt;
        m_gyroSim = new AnalogGyroSim(dt.odometry.m_gyro);
        m_leftEncoderSim = new EncoderSim(dt.odometry.leftEncoder);
        m_rightEncoderSim = new EncoderSim(dt.odometry.rightEncoder);
        m_drivetrainSystem =
                LinearSystemId.identifyDrivetrainSystem(
                        dt.config.kVLinear,
                        dt.config.kALinear,
                        dt.config.kVAngular,
                        dt.config.kAAngular,
                        dt.config.kTrackWidth);
        m_drivetrainSimulator =
                new DifferentialDrivetrainSim(
                        m_drivetrainSystem,
                        DCMotor.getRomiBuiltIn(1),
                        1,
                        dt.config.kTrackWidth,
                        dt.config.kWheelRadius,
                        null);
    }

    /** Reset all sim devices to 0 */
    public void resetSimOdometry() {
        m_drivetrainSimulator.setPose(new Pose2d());
    }

    /** Update our simulation. This should be run every robot loop in simulation. */
    public void simulationPeriodic() {
        // To update our simulation, we set motor voltage inputs, update the
        // simulation, and write the simulated positions and velocities to our
        // simulated encoder and gyro. We negate the right side so that positive
        // voltages make the right side move forward.
        if (!Robot.romiConnected
                && !DriverStation.isDisabled()) { // Only update these if romi isn't connected
            m_drivetrainSimulator.setInputs(
                    dt.leftMotor.get() * RobotController.getInputVoltage(),
                    dt.rightMotor.get() * RobotController.getInputVoltage());
            m_drivetrainSimulator.update(0.02);

            m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
            m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
            m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
            m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
            m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
        }
    }
}
