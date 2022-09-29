package frc.robot.leds.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/** All of the commands to schedule LEDs */
public class LEDCommands {

    public static void setupDefaultCommand() {
        Robot.leds.scheduler.setDefaultAnimation(
                "Default LEDs",
                new BlinkLEDCommand(Color.kPurple).withName("Default LEDs"),
                20,
                -101);
    }

    public static Command blink(Color color, String name, int priority, int timeout) {
        return new ScheduleAnimation(
                name, new BlinkLEDCommand(color).withName(name), priority, timeout);
    }

    public static Command rainbow(String name, int priority, int timeout) {
        return new ScheduleAnimation(
                name, new RainbowLEDCommand(20).withName(name), priority, timeout);
    }

    public static Command solidColor(Color color, String name, int priority, int timeout) {
        return new ScheduleAnimation(
                name, new SetLEDToRGBCommand(color).withName(name), priority, timeout);
    }

    public static Command snowfall(String name, int priority, int timeout) {
        return new ScheduleAnimation(
                name, new SnowfallLEDCommand(100).withName(name), priority, timeout);
    }
}
