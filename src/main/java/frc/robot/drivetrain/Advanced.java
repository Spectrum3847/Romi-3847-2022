// Adavnced Controls
// PID, Kinematics, Velocity controlers, etc
package frc.robot.drivetrain;

import static frc.robot.drivetrain.DrivetrainConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotController;

/** Add your docs here. */
public class Advanced {

    private Drivetrain dt;

    public final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kVLinear);

    public final DifferentialDriveKinematics kinematics =
            new DifferentialDriveKinematics(kTrackWidth);

    public final PIDController leftPid = new PIDController(kPLeft, 0, 0);
    public final PIDController rightPid = new PIDController(kPRight, 0, 0);
    public final RamseteController ramseteController = new RamseteController(ramseteB, ramseteZeta);

    public Advanced(Drivetrain dt) {
        this.dt = dt;
    }

    // DriveSpeeds takes xSpeed in meters per sec, and rot in radians per sec
    public void driveSpeeds(double xSpeed, double rot) {
        var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
        setSpeeds(wheelSpeeds);
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        var leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
        var rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

        var leftOutput =
                leftPid.calculate(dt.odometry.leftEncoder.getRate(), speeds.leftMetersPerSecond);
        var rightOutput =
                rightPid.calculate(dt.odometry.rightEncoder.getRate(), speeds.rightMetersPerSecond);

        dt.leftMotor.setVoltage((leftOutput + leftFeedforward));
        dt.rightMotor.setVoltage((rightOutput + rightFeedforward)); // negate right side

        dt.diffDrive.feed();
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        var batteryVoltage = RobotController.getBatteryVoltage();
        if (Math.max(Math.abs(leftVolts), Math.abs(rightVolts)) > batteryVoltage) {
            leftVolts *= batteryVoltage / 12.0;
            rightVolts *= batteryVoltage / 12.0;
        }
        dt.leftMotor.setVoltage(leftVolts);
        dt.rightMotor.setVoltage(rightVolts);
        dt.diffDrive.feed();
    }
}
