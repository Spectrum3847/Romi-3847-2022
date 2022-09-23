package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class RobotConstants {

    public static final String Canivore = "3847";

    public static final class MotorIDs {
        public static final int driveMotorLeft = 0;
        public static final int driveMotorRight = 1;
        public static final int elevatorMotor = 9;
        public static final int intakeMotor = 20;
    }

    public static final class SolenoidPorts {
        public static final int kIntakeDown = 0;
    }
}
