package frc.robot.intakeExample;

import edu.wpi.first.wpilibj.Timer;
import frc.SpectrumLib.telemetry.TelemetrySubsystem;

/** Add your docs here. */
public class IntakeTelemetry extends TelemetrySubsystem {
    private Intake intake;
    private double speed = 0;

    protected double updateRate = 0.2;

    public IntakeTelemetry(Intake intake) {
        super("Intake");
        this.intake = intake;
        tab.addNumber("speed", () -> getSpeed()).withPosition(0, 0).withSize(2, 1);
        TelemetryThread.start();
        setUpdateRate(1);
    }

    private double getSpeed() {
        return speed;
    }

    protected void update() {
        speed = intake.getRPM();
    }

    protected void setUpdateRate(double rate) {
        updateRate = rate;
    }

    Thread TelemetryThread =
            new Thread(
                    new Runnable() {
                        public void run() {
                            while (true) {
                                if (Timer.getFPGATimestamp() > 5) {
                                    update();
                                }
                                Timer.delay(updateRate); // Loop runs at 5hz
                            }
                        }
                    });
}
