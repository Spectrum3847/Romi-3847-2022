package frc.robot.intakeExample;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;

/** Commands that are run on the example Intake */
public class IntakeCommands {

    /** Setup the Default Commands for the Subsystem */
    public static void setupDefaultCommand() {
        Robot.intake.setDefaultCommand(stop());
        Robot.intake.pneumatic.setDefaultCommand(up());
    }

    /** Creates a parrellel command group so the intake goes down and intakes */
    public static Command intakePieces() {
        return down().alongWith(spinIn()).withName("Intake Pieces");
    }

    /** Run the intake motor in manual output */
    public static Command run(double speed) {
        RunCommand run = new RunCommand(() -> Robot.intake.setManualOutput(speed), Robot.intake);
        run.setName("Run Intake");
        return run;
    }

    /** The default speed for running the intake to collect gamepieces */
    public static Command spinIn() {
        return run(1);
    }

    /** The default speed for ejecting game pieces */
    public static Command eject() {
        return run(-0.8);
    }

    /** Stop the intake */
    public static Command stop() {
        return new RunCommand(() -> Robot.intake.stop(), Robot.intake).withName("Stop Intake");
    }

    /** Raise the intake */
    public static Command up() {
        return new RunCommand(() -> Robot.intake.up(), Robot.intake.pneumatic);
    }

    /** Lower the intake */
    public static Command down() {
        return new RunCommand(() -> Robot.intake.down(), Robot.intake.pneumatic);
    }
}
