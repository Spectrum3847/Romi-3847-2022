package frc.robot.intakeExample;

/**
 * This is a blank Intake class to be used if the robot doesn't have an intake subsystem attached
 */
public class IntakeNull extends Intake {
    public IntakeConfig config;

    public IntakeNull() {
        config = new IntakeConfig("IntakeNull");
    }

    public void setManualOutput(double speed) {}

    public void stop() {}

    public void down() {}

    public void up() {}
}
