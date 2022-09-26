package frc.robot.drivetrain;

import frc.robot.Robot;

/** Add your docs here. */
public class DrivetrainConfig {
    public final double kMaxSpeed = 0.6;
    public final double kMaxAccel = 0.4;
    public final double kMaxAngularSpeed = Math.PI / 4;

    public final double kCountsPerRevolution = 1440.0;
    public final double kWheelRadius = 0.035; // 70mm diameter meters / 2 for radius
    public final double kTrackWidth = 0.14;

    public final double kPLeft = 13;
    public final double kPRight = 13;

    public final double ramseteB = 2.1;
    public final double ramseteZeta = 0.8;

    public final double kS = 0.38069;
    public final double kVLinear = 9.5975;
    public final double kALinear = 0.60273;
    public final double kVAngular = 20;
    public final double kAAngular = 1;

    public DrivetrainConfig() {
        switch (Robot.config.getRobotType()) {
            case COMP:
                // Set all the constants specifically for the competition robot
                break;
            case PRACTICE:
                // Set all the constants specifically for the practice robot
                break;
            case SIM:
                // Set all the constants specifically for the simulation
                break;
        }
    }
}
