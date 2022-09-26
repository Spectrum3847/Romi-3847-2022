// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.SpectrumLib.telemetry.TelemetrySubsystem;
import frc.robot.auton.AutonSetup;

/** Add your docs here. */
public class RobotTelemetry extends TelemetrySubsystem {

    public static ComplexWidget autonSelectorWidget;

    public RobotTelemetry() {
        super();
        // Allows us to see all running commands on the robot
        SmartDashboard.putData(CommandScheduler.getInstance());

        // Setup the auton selector to display on shuffleboard
        AutonSetup.setupSelectors();
        autonSelectorWidget =
                mainTab.add("Auton Selection", AutonSetup.autonChooser)
                        .withPosition(4, 0)
                        .withSize(3, 1);

        // Add Romi Connected Display to shuffelboard
        mainTab.addBoolean("Romi Connected", () -> Robot.isRomiConnected()).withPosition(0, 0);
    }
}
