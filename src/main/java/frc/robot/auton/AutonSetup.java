package frc.robot.auton;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.auton.commands.AutonDistance;
import frc.robot.auton.commands.AutonTime;

public class AutonSetup {
    public static final SendableChooser<Command> autonChooser = new SendableChooser<>();

    //AutoRoutines
    private static final Command autonTime = new AutonTime();
    private static final Command autonDistance = new AutonDistance();

    // A chooser for autonomous commands

    public static void setupSelectors() {
        autonChooser.setDefaultOption("AutonTime", autonTime);
        autonChooser.addOption("autonDistance", autonDistance);
        autonChooser.addOption("Nothing", new PrintCommand("DO NOTHING AUTON RUNNING"));
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
