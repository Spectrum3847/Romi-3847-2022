// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.leds.commands;

import frc.robot.Robot;

/** Add your docs here. */
public class LEDCommands {

    public static void setupDefaultCommand(){
        Robot.leds.setDefaultCommand(new BlinkLEDCommand(Robot.leds, 500, 60, 0, 100));
    }
}
