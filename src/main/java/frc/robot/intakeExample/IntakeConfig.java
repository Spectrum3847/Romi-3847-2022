package frc.robot.intakeExample;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import frc.SpectrumLib.subsystems.rollerMech.RollerMechConfig;
import frc.robot.Robot;

/** Constants for the intake */
public class IntakeConfig extends RollerMechConfig {
    int follerID = Robot.config.motors.intakeFollower;

    public IntakeConfig(String name) {
        super(name);
        name = "Intake";

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

        this.diameterInches = 2.0;
        this.gearRatio = 30 / 12; // 30t large pulley, 12t small pulley

        /* Control Loop Constants */
        kP = 0.0;
        kI = 0.0;
        kD = 0.0;
        kF = 0.0;
        kIz = 150;
        motionCruiseVelocity = 0;
        motionAcceleration = 0;

        /* Current Limiting */
        currentLimit = 40;
        tirggerThresholdLimit = 45;
        PeakCurrentDuration = 0.5;
        EnableCurrentLimit = true;
        supplyLimit =
                new SupplyCurrentLimitConfiguration(
                        EnableCurrentLimit,
                        currentLimit,
                        tirggerThresholdLimit,
                        PeakCurrentDuration);

        updateTalonFXConfig();
    }
}
