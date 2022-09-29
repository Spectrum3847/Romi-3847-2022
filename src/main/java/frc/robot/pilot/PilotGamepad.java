package frc.robot.pilot;

import edu.wpi.first.wpilibj.util.Color;
import frc.SpectrumLib.gamepads.Gamepad;
import frc.SpectrumLib.gamepads.mapping.ExpCurve;
import frc.robot.elevatorSim.ElevatorCommands;
import frc.robot.intakeExample.IntakeCommands;
import frc.robot.leds.commands.LEDCommands;

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
        gamepad.bButton.whileHeld(
                IntakeCommands.intakePieces()
                        .alongWith(LEDCommands.blink(Color.kYellow, "Yellow", 10, 5)));
    }

    public void setupDisabledButtons() {
        gamepad.aButton.whileHeld(LEDCommands.solidColor(Color.kGreen, "Green", 5, 5));
        gamepad.bButton.whileHeld(LEDCommands.blink(Color.kBlue, "Blink Blue", 10, 5));
        gamepad.xButton.whileHeld(LEDCommands.rainbow("Rainbow", 15, 5));
        gamepad.yButton.whileHeld(LEDCommands.snowfall("Snowfall", 20, 5));
    }

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
