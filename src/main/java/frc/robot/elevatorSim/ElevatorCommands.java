// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevatorSim;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;

/** Add your docs here. */
public class ElevatorCommands {

    public static void setupDefaultCommand(){
        Robot.elevator.setDefaultCommand(stop());
    }

    public static Command goToHeight(int height){
        return new RunCommand(() -> Robot.elevator.goToTarget(height), Robot.elevator);
    }

    public static Command stop(){
        return new RunCommand(() -> Robot.elevator.stop(), Robot.elevator);
    }
}
