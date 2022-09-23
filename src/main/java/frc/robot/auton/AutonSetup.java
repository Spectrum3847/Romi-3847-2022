package frc.robot.auton;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Robot;
import frc.robot.auton.commands.AutonCommands;
import frc.robot.auton.commands.AutonDistance;
import frc.robot.auton.commands.AutonTime;
import frc.robot.auton.commands.FiveBall;

public class AutonSetup {
    public static final SendableChooser<Command> autonChooser = new SendableChooser<>();

    // A chooser for autonomous commands
    public static void setupSelectors() {
        autonChooser.setDefaultOption("3mFWD", AutonCommands.followIntialPath("3mFWD"));
        autonChooser.addOption("AutonTime", new AutonTime());
        autonChooser.addOption("autonDistance", new AutonDistance());
        autonChooser.addOption("Nothing", new PrintCommand("DO NOTHING AUTON RUNNING"));
        autonChooser.addOption("Five_Ball", new FiveBall());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public static Command getAutonomousCommand() {
        // return new CharacterizeLauncher(Robot.launcher);
        return autonChooser.getSelected();
    }
}
