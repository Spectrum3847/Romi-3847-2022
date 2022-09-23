package frc.robot.intakeExample;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import frc.lib.subsystems.RollerConstants;

/** Add your docs here. */
public class IntakeConstants extends RollerConstants {

    public static final double kElevatorGearing = 10.0;

    public IntakeConstants() {
        name = "Intake";

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

        setConfig();
    }
}
