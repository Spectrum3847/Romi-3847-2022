// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevatorSim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    // Standard classes for controlling our elevator
    public final PIDController m_controller =
            new PIDController(ElevatorConstants.kElevatorKp, 0, 0);
    public final Encoder m_encoder =
            new Encoder(ElevatorConstants.kEncoderAChannel, ElevatorConstants.kEncoderBChannel);
    public final PWMSparkMax m_motor = new PWMSparkMax(ElevatorConstants.kMotorPort);
    public final ElevatorSimulator elevatorSim;

    /** Creates a new ElevatorSim. */
    public Elevator() {
        m_encoder.setDistancePerPulse(ElevatorConstants.kElevatorEncoderDistPerPulse);

        // Used for simulation
        elevatorSim = new ElevatorSimulator(this);
    }

    public void setOutput(double speed) {
        m_motor.set(speed);
    }

    public void goToTarget(int height) {
        double pidOutput = m_controller.calculate(m_encoder.getDistance(), height);
        m_motor.setVoltage(pidOutput);
    }

    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        elevatorSim.simulationPeriodic();
    }
}
