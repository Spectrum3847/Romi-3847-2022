package frc.robot.leds.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class LEDCommands {

    public static void setupDefaultCommand() {}

    public static Command blink(Color color, String name, int priority, int timeout) {
        return new ScheduleAnimation(
                "Blink " + name,
                new BlinkLEDCommand(color).withName("Blink " + name),
                priority,
                timeout);
    }

    public static Command rainbow(String name, int priority, int timeout) {
        return new ScheduleAnimation(
                "Rainbow " + name,
                new RainbowLEDCommand(20).withName("Rainbow " + name),
                priority,
                timeout);
    }

    public static Command solidColor(Color color, String name, int priority, int timeout) {
        return new ScheduleAnimation(
                "Solid Color " + name,
                new SetLEDToRGBCommand(color).withName("Solid Color " + name),
                priority,
                timeout);
    }

    public static Command snowfall(String name, int priority, int timeout) {
        return new ScheduleAnimation(
                "Snowfall " + name,
                new SnowfallLEDCommand(100).withName("Snowfall " + name),
                priority,
                timeout);
    }
}
