// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevatorSim;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  // distance per pulse = (distance per revolution) / (pulses per revolution)
  // = (Pi * D) / ppr
  private static final double kElevatorEncoderDistPerPulse = 2.0 * Math.PI * ElevatorConstants.kElevatorDrumRadius / 4096;

  private final DCMotor m_elevatorGearbox = DCMotor.getVex775Pro(4);

  // Standard classes for controlling our elevator
  private final PIDController m_controller = new PIDController(ElevatorConstants.kElevatorKp, 0, 0);
  private final Encoder m_encoder = new Encoder(ElevatorConstants.kEncoderAChannel, ElevatorConstants.kEncoderBChannel);
  private final PWMSparkMax m_motor = new PWMSparkMax(ElevatorConstants.kMotorPort);

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim = new ElevatorSim(
      m_elevatorGearbox,
      ElevatorConstants.kElevatorGearing,
      ElevatorConstants.kCarriageMass,
      ElevatorConstants.kElevatorDrumRadius,
      ElevatorConstants.kMinElevatorHeight,
      ElevatorConstants.kMaxElevatorHeight,
      VecBuilder.fill(0.01));
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(
      new MechanismLigament2d(
          "Elevator", Units.metersToInches(m_elevatorSim.getPositionMeters()), 90));

  /** Creates a new ElevatorSim. */
  public Elevator() {
    m_encoder.setDistancePerPulse(kElevatorEncoderDistPerPulse);

    // Publish Mechanism2d to SmartDashboard
    SmartDashboard.putData("Elevator Sim", m_mech2d);
  }

  public void setOutput(double speed){
    m_motor.set(speed);
  }

  public void goToTarget(int height){
    double pidOutput = m_controller.calculate(m_encoder.getDistance(), height);
    m_motor.setVoltage(pidOutput);
  }

  public void stop(){
    m_motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

    // Update elevator visualization with simulated position
    m_elevatorMech2d.setLength(Units.metersToInches(m_elevatorSim.getPositionMeters()));
  }
}
