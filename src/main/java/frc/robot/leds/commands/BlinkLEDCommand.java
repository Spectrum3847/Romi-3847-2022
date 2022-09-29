package frc.robot.leds.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.SpectrumLib.util.Util;
import frc.robot.Robot;
import frc.robot.leds.LEDs;

public class BlinkLEDCommand extends CommandBase {
    LEDs ledSubsystem;
    long commandScheduleTime;
    long startTime;
    int waitTime; // Wait time in milliseconds
    int r, g, b;
    boolean on = true;

    public BlinkLEDCommand(int waitTimeMS, int r, int g, int b) {
        this.ledSubsystem = Robot.leds;
        this.startTime = (long) Units.secondsToMilliseconds(Util.getTime());
        this.waitTime = waitTimeMS;
        this.r = r;
        this.g = g;
        this.b = b;

        addRequirements(ledSubsystem);
    }

    public BlinkLEDCommand(int waitTime, Color color) {
        this(
                waitTime,
                new Color8Bit(color).red,
                new Color8Bit(color).green,
                new Color8Bit(color).blue);
    }

    public BlinkLEDCommand(Color color) {
        this(500, color);
    }

    public boolean runsWhenDisabled() {
        return true;
    }

    private long getTime() {
        return (long) Units.secondsToMilliseconds(Util.getTime());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
            ledSubsystem.setRGB(i, r, g, b);
        }
        ledSubsystem.sendData();
        commandScheduleTime = getTime();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (getTime() - startTime >= waitTime) {
            if (on) {
                for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
                    ledSubsystem.setRGB(i, 0, 0, 0);
                }
                ledSubsystem.sendData();
                on = false;
            } else {
                for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
                    ledSubsystem.setRGB(i, r, g, b);
                }
                ledSubsystem.sendData();
                on = true;
            }
            startTime = getTime();
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
