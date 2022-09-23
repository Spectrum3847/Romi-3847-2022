// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intakeExample;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.lib.sim.PhysicsSim;
import frc.lib.subsystems.RollerSubsystem;
import frc.lib.subsystems.SolenoidSubsystem;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.MotorIDs;
import frc.robot.RobotConstants.SolenoidPorts;

/** Intake Mechanism subsyste */
public class Intake extends RollerSubsystem {
    // Creates an instance of IntakeConstants this is where we configure much of the motor
    public IntakeConstants constants = new IntakeConstants();

    // Creates a pneumatic subsystem inside of the intake, this is ued for moving the intake up and
    // down
    public final SolenoidSubsystem pneumatic;

    public Intake() {
        setName(constants.name); // Sets the name of the subsystem used for some dashboard things
        configureMotors();

        // Intialize the Solenoid subsytem with name and it's port form RobotConstants SolenoidPorts
        pneumatic = new SolenoidSubsystem(constants.name + "Solenoid", SolenoidPorts.kIntakeDown);
    }

    protected void configureMotors() {
        // Intializes the Falcon getting it's ID from MotorIDs in constants and the Canivore name
        motorLeader = new WPI_TalonFX(MotorIDs.intakeMotor, RobotConstants.Canivore);

        // Create a physicsSim so encoder values update in simulation
        PhysicsSim.getInstance().addTalonFX(motorLeader, 0.75, 5100.0);

        // Setups the falcon using the method we have in RollerConstants to do this for us.
        constants.setupFalconLeader(motorLeader);
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
