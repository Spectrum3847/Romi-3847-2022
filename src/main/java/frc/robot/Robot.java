package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.SpectrumLib.sim.PhysicsSim;
import frc.SpectrumLib.util.Network;
import frc.robot.auton.AutonSetup;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.DrivetrainCommands;
import frc.robot.elevatorSim.Elevator;
import frc.robot.elevatorSim.ElevatorCommands;
import frc.robot.intakeExample.Intake;
import frc.robot.intakeExample.IntakeCommands;
import frc.robot.intakeExample.IntakeNull;
import frc.robot.leds.LEDs;
import frc.robot.leds.commands.LEDCommands;
import frc.robot.onBoardIO.OnBoardIO;
import frc.robot.onBoardIO.OnBoardIO.ChannelMode;
import frc.robot.pilot.PilotCommands;
import frc.robot.pilot.PilotGamepad;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static RobotConfig config;
    public static RobotTelemetry telemetry;
    public static Drivetrain drivetrain;
    public static OnBoardIO onboardIO;
    public static LEDs leds;
    public static PilotGamepad pilotGamepad;
    public static Elevator elevator;
    public static Intake intake;

    public static String MAC = "";

    // Intialize subsystems and run their setupDefaultCommand methods here
    private void intializeSubsystems() {
        config = new RobotConfig();

        // Setup Subsystems that are different on COMP/PRACTICE/SIM
        switch (config.getRobotType()) {
            case PRACTICE:
                intake = new IntakeNull();
                break;
            default:
                intake = new Intake();
                break;
        }

        drivetrain = new Drivetrain();
        elevator = new Elevator();
        leds = new LEDs();
        onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);
        pilotGamepad = new PilotGamepad();
        telemetry = new RobotTelemetry();

        // Set Default Commands, this method should exist for each subsystem that has commands
        DrivetrainCommands.setupDefaultCommand();
        ElevatorCommands.setupDefaultCommand();
        IntakeCommands.setupDefaultCommand();
        LEDCommands.setupDefaultCommand();
        PilotCommands.setupDefaultCommand();
    }

    public static void resetCommandsAndButtons() {
        CommandScheduler.getInstance().cancelAll(); // Disable any currently running commands
        CommandScheduler.getInstance().clearButtons();
        LiveWindow.setEnabled(false); // Disable Live Window we don't need that data being sent
        LiveWindow.disableAllTelemetry();

        // Reset Config for all gamepads and other button bindings
        pilotGamepad.resetConfig();
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        /* Set the MAC Address for this robot, useful for adjusting comp/practice bot settings*/
        MAC = Network.getMACaddress();
        Shuffleboard.getTab("Robot"); // Makes the Robot tab the first tab on the Shuffleboard
        intializeSubsystems();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        resetCommandsAndButtons();
    }

    @Override
    public void disabledPeriodic() {
        CommandScheduler.getInstance().enable();
    }

    @Override
    public void autonomousInit() {
        resetCommandsAndButtons();

        Command autonCommand = AutonSetup.getAutonomousCommand();
        if (autonCommand != null) {
            autonCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        resetCommandsAndButtons();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        resetCommandsAndButtons();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when a simulation starts */
    public void simulationInit() {}

    /** This function is called periodically during a simulation */
    public void simulationPeriodic() {
        PhysicsSim.getInstance().run();
    }

    // Romi only code checks if the simulator is geeting update accel values from
    // romi to see if it's connected
    // There is probably a much better way to do this
    private static double lastAccelValue = 0;
    public static boolean romiConnected = false;
    private static int counter = 11;

    public static boolean isRomiConnected() {
        if (counter > 20) {
            double currentAccel = drivetrain.odometry.getAccelZ();
            romiConnected = currentAccel != lastAccelValue;
            lastAccelValue = currentAccel;
            counter = 0;
        }
        counter++;
        return romiConnected;
    }
}
