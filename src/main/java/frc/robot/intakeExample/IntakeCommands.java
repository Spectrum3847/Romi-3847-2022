// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intakeExample;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;

/** Add your docs here. */
public class IntakeCommands {

    /**Setup the Default Commands for the Subsystem */
    public static void setupDefaultCommand(){
        Robot.intake.setDefaultCommand(stop());
        Robot.intake.pneumatic.setDefaultCommand(up());
    }

    /** Creates a parrellel command group so the intake goes down and intakes */
    public static Command intakePieces(){
        return down().alongWith(spinIn());
    }

    /** Run the intake motor in manual output */
    public static Command run(double speed) {
        return new RunCommand(() -> Robot.intake.setManualOutput(speed), Robot.intake);
    }

    /** The default speed for running the intake to collect gamepieces */
    public static Command spinIn() {
        return run(1);
    }

    /** The default speed for ejecting game pieces */
    public static Command eject(){
        return run(-0.8);
    }

    /**Stop the intake */
    public static Command stop() {
        return new RunCommand(() -> Robot.intake.stop(), Robot.intake);
    }

    /**Raise the intake */
    public static Command up(){
        return new RunCommand(() -> Robot.intake.up(), Robot.intake.pneumatic);
    }

    /**Lower the intake */
    public static Command down(){
        return new RunCommand(() -> Robot.intake.down(), Robot.intake.pneumatic);
    }
}
