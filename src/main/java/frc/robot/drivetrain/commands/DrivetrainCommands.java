package frc.robot.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;

/** Add your docs here. */
public class DrivetrainCommands {

    public static void setupDefaultCommand() {
        Robot.drivetrain.setDefaultCommand(
                new RunCommand(
                        () ->
                                Robot.drivetrain.arcadeDrive(
                                        Robot.pilotGamepad.getDriveThrottle(),
                                        Robot.pilotGamepad.getDriveSteering()),
                        Robot.drivetrain));
    }

    public static Command stop() {
        return new RunCommand(() -> Robot.drivetrain.stop(), Robot.drivetrain);
    }
}
