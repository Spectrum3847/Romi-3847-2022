package frc.robot.pilot;

import frc.lib.gamepads.Gamepad;
import frc.lib.gamepads.mapping.ExpCurve;
import frc.robot.elevatorSim.ElevatorCommands;
import frc.robot.intakeExample.IntakeCommands;

/** Used to add buttons to the pilot gamepad and configure the joysticks */
public class PilotGamepad extends Gamepad {
    public static ExpCurve throttleCurve =
            new ExpCurve(
                    PilotConstants.throttleExp,
                    0,
                    PilotConstants.throttleScaler,
                    PilotConstants.throttleDeadband);
    public static ExpCurve steeringCurve =
            new ExpCurve(
                    PilotConstants.steeringExp,
                    0,
                    PilotConstants.steeringScaler,
                    PilotConstants.steeringDeadband);

    public PilotGamepad() {
        super("Pilot", PilotConstants.port);
    }

    public void setupTeleopButtons() {
        gamepad.aButton.whileHeld(ElevatorCommands.goToHeight(30));
        gamepad.bButton.whileHeld(IntakeCommands.intakePieces());
    }

    public void setupDisabledButtons() {}

    public void setupTestButtons() {}

    public double getDriveThrottle() {
        return throttleCurve.calculateMappedVal(this.gamepad.leftStick.getY());
    }

    public double getDriveSteering() {
        return steeringCurve.calculateMappedVal(this.gamepad.rightStick.getX());
    }

    public void rumble(double intensity) {
        this.gamepad.setRumble(intensity, intensity);
    }
}
