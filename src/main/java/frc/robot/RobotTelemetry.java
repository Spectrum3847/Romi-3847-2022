// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import frc.lib.telemetry.TelemetrySubsystem;
import frc.robot.auton.AutonSetup;

/** Add your docs here. */
public class RobotTelemetry extends TelemetrySubsystem {

    public static ComplexWidget autonSelectorWidget;

    public RobotTelemetry() {
        super();
        AutonSetup.setupSelectors();
        autonSelectorWidget =
                mainTab.add("Auton Selection", AutonSetup.autonChooser)
                        .withPosition(4, 0)
                        .withSize(3, 1);
        mainTab.addBoolean("Romi Connected", () -> Robot.isRomiConnected()).withPosition(0, 0);
    }
}
