package frc.robot.auton.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drivetrain.commands.DriveDistance;
import frc.robot.drivetrain.commands.TurnDegrees;

public class AutonDistance extends SequentialCommandGroup {
    /**
     * Creates a new Autonomous Drive based on distance. This will drive out for a specified
     * distance, turn around and drive back.
     *
     * @param drivetrain The drivetrain subsystem on which this command will run
     */
    public AutonDistance() {
        addCommands(
                new DriveDistance(-0.5, 10),
                new TurnDegrees(-0.5, 180),
                new DriveDistance(-0.5, 10),
                new TurnDegrees(0.5, 180));
    }
}
