package frc.robot.intakeExample;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.SpectrumLib.sim.PhysicsSim;
import frc.SpectrumLib.subsystems.FollowerFalcon;
import frc.SpectrumLib.subsystems.pneumatic.PneumaticSubsystem;
import frc.SpectrumLib.subsystems.rollerMech.RollerMechSubsystem;
import frc.robot.Robot;

/** Intake Mechanism subsyste */
public class Intake extends RollerMechSubsystem {
    // Creates an instance of IntakeConstants this is where we configure much of the motor
    public IntakeConfig config;
    protected FollowerFalcon follower1;

    // Creates a pneumatic subsystem inside of the intake, this is ued for moving the intake up and
    // down
    public final PneumaticSubsystem pneumatic;

    public Intake() {
        this(new IntakeConfig("Intake"));
    }

    private Intake(IntakeConfig config) {
        super(config);
        this.config = config;
        configureMotors();

        // Intialize the Solenoid subsytem with name and it's port form Robot.constants
        pneumatic =
                new PneumaticSubsystem(config.name + "Pneumatic", Robot.config.pneumatic.intake);
    }

    protected void configureMotors() {
        // Intializes the Falcon getting it's ID from Robot.config and the Canivore name
        motorLeader = new WPI_TalonFX(Robot.config.motors.intakeMotor, Robot.config.Canivore);
        FollowerFalcon =
                new FollowerFalcon(
                        config.follerID,
                        Robot.config.Canivore,
                        config.TalonFXConfig,
                        config.kFollowerInverted,
                        motorLeader);
        // Create a physicsSim so encoder values update in simulation
        PhysicsSim.getInstance().addTalonFX(motorLeader, 0.75, 5100.0);

        // Setups the falcon
        setupFalconLeader();
    }

    /** Have the pneumatic solenoid lower the intake */
    public void down() {
        pneumatic.on();
    }

    /** Have the pneumatic solenoid raise the intake */
    public void up() {
        pneumatic.off();
    }
}
