package frc.robot.elevatorSim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    public ElevatorConfig config;

    // Standard classes for controlling our elevator
    public final PIDController m_controller;
    public final Encoder m_encoder;
    public final PWMSparkMax m_motor;
    public final ElevatorSimulator elevatorSim;

    /** Creates a new ElevatorSim. */
    public Elevator() {
        config = new ElevatorConfig();
        m_controller = new PIDController(config.kElevatorKp, 0, 0);
        m_encoder = new Encoder(config.kEncoderAChannel, config.kEncoderBChannel);
        m_motor = new PWMSparkMax(config.kMotorPort);

        m_encoder.setDistancePerPulse(config.kElevatorEncoderDistPerPulse);

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
