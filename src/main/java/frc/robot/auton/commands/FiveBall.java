package frc.robot.auton.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Add your docs here. */
public class FiveBall extends SequentialCommandGroup {
    public FiveBall() {
        addCommands(
                AutonCommands.followIntialPath("Five_FirstBalls"),
                AutonCommands.followPath("Five_2ndBall"),
                AutonCommands.followPath("Five_Intake2ndBall"));
    }
}
