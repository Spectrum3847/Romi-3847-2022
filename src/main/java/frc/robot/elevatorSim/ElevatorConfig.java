package frc.robot.elevatorSim;

import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

/** Add your docs here. */
public class ElevatorConfig {
    public final int kMotorPort = Robot.config.motors.elevatorMotor;
    public final int kEncoderAChannel = 8;
    public final int kEncoderBChannel = 9;

    public final double kElevatorKp = 5.0;
    public final double kElevatorGearing = 10.0;
    public final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    public final double kCarriageMass = 4.0; // kg

    public final int kMinElevatorHeight = (int) Units.inchesToMeters(2);
    public final int kMaxElevatorHeight = (int) Units.inchesToMeters(50);

    // distance per pulse = (distance per revolution) / (pulses per revolution)
    // = (Pi * D) / ppr
    public final double kElevatorEncoderDistPerPulse = 2.0 * Math.PI * kElevatorDrumRadius / 4096;

    public ElevatorConfig() {
        switch (Robot.config.getRobotType()) {
            case COMP:
                // Set all the constants specifically for the competition robot
                break;
            case PRACTICE:
                // Set all the constants specifically for the practice robot
                break;
            case SIM:
                // Set all the constants specifically for the simulation
                break;
        }
    }
}
