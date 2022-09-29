package frc.robot.leds.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.leds.LEDs;

public class SnowfallLEDCommand extends CommandBase {
    /** Creates a new SnowfallLEDCommand. */
    private final LEDs ledSubsystem;

    long waitTime;
    long startTime;
    int stage;

    public SnowfallLEDCommand(long waitTime) {
        ledSubsystem = Robot.leds;
        this.waitTime = waitTime;
        this.startTime = System.currentTimeMillis();
        stage = 0;
        addRequirements(ledSubsystem);
    }

    public boolean runsWhenDisabled() {
        return true;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
            if (i % 4 == stage) {
                ledSubsystem.setRGB(i, 255, 255, 255);
                continue;
            }
            ledSubsystem.setRGB(i, 20, 120, 255);
        }
        ledSubsystem.sendData();
        stage++;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (System.currentTimeMillis() - startTime >= waitTime) {
            for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
                if (i % 4 == stage) {
                    ledSubsystem.setRGB(i, 255, 255, 255);
                    continue;
                }
                ledSubsystem.setRGB(i, 20, 120, 255);
            }
            ledSubsystem.sendData();
            stage = stage + 1 > 3 ? 0 : stage + 1;
            startTime = System.currentTimeMillis();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
