// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevatorSim;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class ElevatorSimulator {
    Elevator elevator;

    private final DCMotor m_elevatorGearbox = DCMotor.getVex775Pro(4);

    // Simulation classes help us simulate what's going on, including gravity.
    private final ElevatorSim m_elevatorSim = new ElevatorSim(
            m_elevatorGearbox,
            ElevatorConstants.kElevatorGearing,
            ElevatorConstants.kCarriageMass,
            ElevatorConstants.kElevatorDrumRadius,
            ElevatorConstants.kMinElevatorHeight,
            ElevatorConstants.kMaxElevatorHeight,
            VecBuilder.fill(0.01));
    private final EncoderSim m_encoderSim;

    // Create a Mechanism2d visualization of the elevator
    private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
    private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(
            new MechanismLigament2d(
                    "Elevator", Units.metersToInches(m_elevatorSim.getPositionMeters()), 90));

    public ElevatorSimulator(Elevator elevator) {
        this.elevator = elevator;
        m_encoderSim = new EncoderSim(elevator.m_encoder);
        // Publish Mechanism2d to SmartDashboard
        SmartDashboard.putData("Elevator Sim", m_mech2d);
    }

    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_elevatorSim.setInput(elevator.m_motor.get() * RobotController.getBatteryVoltage());

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
