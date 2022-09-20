package frc.robot.auton;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;
import frc.robot.auton.commands.AutonDistance;
import frc.robot.auton.commands.AutonTime;
import frc.robot.auton.commands.FollowTrajectory;
import frc.robot.drivetrain.DrivetrainConstants;
import frc.robot.drivetrain.commands.DrivetrainCommands;

public class AutonSetup {
    public static final SendableChooser<Command> autonChooser = new SendableChooser<>();

    //AutoRoutines
    private static final Command autonTime = new AutonTime();
    private static final Command autonDistance = new AutonDistance();
    private static final Trajectory examplePath = PathPlanner.loadPath("3mFWD", DrivetrainConstants.kMaxSpeed,
    DrivetrainConstants.kMaxAccel);
    private static final Command oneMeter = new FollowTrajectory(examplePath).andThen(DrivetrainCommands.stop());
    
    // A chooser for autonomous commands
    public static void setupSelectors() {
        autonChooser.setDefaultOption("AutonTime", autonTime);
        autonChooser.addOption("autonDistance", autonDistance);
        autonChooser.addOption("Nothing", new PrintCommand("DO NOTHING AUTON RUNNING"));
        autonChooser.addOption("oneMeter", oneMeter);
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
