// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.driverGamepad;

import frc.lib.gamepads.Gamepad;
import frc.lib.gamepads.mapping.ExpCurve;
import frc.robot.elevatorSim.ElevatorCommands;

/** Add your docs here. */
public class DriverGamepad extends Gamepad{
    public static ExpCurve throttleCurve = new ExpCurve(DriverConstants.throttleExp, 
                                                            0, DriverConstants.throttleScaler, 
                                                            DriverConstants.throttleDeadband);
    public static ExpCurve steeringCurve = new ExpCurve(DriverConstants.steeringExp, 
                                                            0, DriverConstants.steeringScaler, 
                                                            DriverConstants.steeringDeadband);

    public DriverGamepad(){
        super("Driver", DriverConstants.port);
    }

    public void setupTeleopButtons(){
        gamepad.aButton.whileHeld(ElevatorCommands.goToHeight(30));
    }
  
    public void setupDisabledButtons(){
  
    }
  
    public void setupTestButtons(){
  
    }

    public double getDriveThrottle(){
        return throttleCurve.calculateMappedVal(this.gamepad.leftStick.getY());
    }

    public double getDriveSteering(){
        return steeringCurve.calculateMappedVal(this.gamepad.rightStick.getX());
    }
}
