package frc.robot.leds.commands;

import frc.robot.Robot;

/** Add your docs here. */
public class LEDCommands {

    public static void setupDefaultCommand() {
        Robot.leds.setDefaultCommand(new BlinkLEDCommand(Robot.leds, 500, 60, 0, 100));
    }
}
