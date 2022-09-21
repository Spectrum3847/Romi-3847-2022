// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.Util;
import frc.robot.auton.AutonSetup;
import frc.robot.driverGamepad.DriverGamepad;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.DrivetrainCommands;
import frc.robot.elevatorSim.Elevator;
import frc.robot.elevatorSim.ElevatorCommands;
import frc.robot.leds.LEDs;
import frc.robot.leds.commands.LEDCommands;
import frc.robot.onBoardIO.OnBoardIO;
import frc.robot.onBoardIO.OnBoardIO.ChannelMode;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static RobotTelemetry telemetry;
  public static Drivetrain drivetrain;
  public static OnBoardIO onboardIO;
  public static LEDs leds;
  public static DriverGamepad driverGamepad;
  public static Elevator elevator;

  //Intialize subsystems and run their setupDefaultCommand methods here
  private void intializeSubsystems() {
    drivetrain = new Drivetrain();
    elevator = new Elevator();
    leds = new LEDs();
    onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);
    driverGamepad = new DriverGamepad();
    telemetry = new RobotTelemetry();

    //Set Default Commands
    DrivetrainCommands.setupDefaultCommand();
    ElevatorCommands.setupDefaultCommand();
    LEDCommands.setupDefaultCommand();
  }

  public static String MAC = "";

  public static void resetCommandsAndButtons() {
    CommandScheduler.getInstance().cancelAll(); // Disable any currently running commands
		CommandScheduler.getInstance().clearButtons();
    LiveWindow.setEnabled(false); // Disable Live Window we don't need that data being sent
    LiveWindow.disableAllTelemetry();

    //Reset Configs for all gamepads and other button bindings
    driverGamepad.resetConfig();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    MAC = Util.getMACaddress(); //Set the MAC Address for this robot, useful for adjusting comp/practice bot settings
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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    resetCommandsAndButtons();
  }

  @Override
  public void disabledPeriodic() {}

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
  public void autonomousPeriodic() {

    
  }

  @Override
  public void autonomousExit(){

  }

  @Override
  public void teleopInit() {
    resetCommandsAndButtons();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit(){

  }

  @Override
  public void testInit() {
    resetCommandsAndButtons();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
  
  //Romi only code checks if the simulator is geeting update accel values from romi to see if it's connected
  //There is probably a much better way to do this
  private static double lastAccelValue = 0;
  public static boolean romiConnected = false;
  private static int counter = 11;
  public static boolean isRomiConnected(){
    if (counter > 20){
      double currentAccel = drivetrain.odometry.getAccelZ();
      romiConnected = currentAccel != lastAccelValue;
      lastAccelValue = currentAccel;
      counter = 0;
    }
    counter++;
    return romiConnected;
  }
}
