//Adavnced Controls
//PID, Kinematics, Velocity controlers, etc
package frc.robot.drivetrain;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotController;

/** Add your docs here. */
public class Advanced {

    private Drivetrain dt;

    public final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DrivetrainConstants.kS,
            DrivetrainConstants.kVLinear);

    public final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
            DrivetrainConstants.kTrackWidth);

    public final PIDController leftPid = new PIDController(DrivetrainConstants.kPLeft, 0, 0);
    public final PIDController rightPid = new PIDController(DrivetrainConstants.kPRight, 0, 0);
    public RamseteController ramseteController = new RamseteController(DrivetrainConstants.ramseteB,
            DrivetrainConstants.ramseteZeta);

    public Advanced(Drivetrain dt) {
        this.dt = dt;
    }

    public void plotTrajectory(Trajectory trajectory) {
        ArrayList<Pose2d> poses = new ArrayList<>();

        for (Trajectory.State pose : trajectory.getStates()) {
            poses.add(pose.poseMeters);
        }

        dt.odometry.fieldSim.getObject("foo").setPoses(poses);
    }

    public void drive(double xSpeed, double rot) {
        var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
        setSpeeds(wheelSpeeds);
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        var leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
        var rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

        var leftOutput = leftPid.calculate(dt.odometry.leftEncoder.getRate(), speeds.leftMetersPerSecond);
        var rightOutput = rightPid.calculate(dt.odometry.rightEncoder.getRate(), speeds.rightMetersPerSecond);

        dt.leftMotor.setVoltage((leftOutput + leftFeedforward));
        dt.rightMotor.setVoltage((rightOutput + rightFeedforward)); // negate right side

        dt.diffDrive.feed();
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
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
