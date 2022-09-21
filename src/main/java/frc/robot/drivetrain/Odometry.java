// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.Conversions;
import frc.robot.Robot;

/** Add your docs here. */
public class Odometry {

    private final Drivetrain drivetrain;
    // The Romi has onboard encoders that are hardcoded
    // to use DIO pins 4/5 and 6/7 for the left and right
    public final Encoder leftEncoder = new Encoder(4, 5);
    public final Encoder rightEncoder = new Encoder(6, 7);

    // Set up the RomiGyro
    public final RomiGyro romiGyro = new RomiGyro();
    public final AnalogGyro m_gyro = new AnalogGyro(0); // Used for simulation

    // Set up the BuiltInAccelerometer
    public final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

    public final Field2d fieldSim = new Field2d();

    // Field Odometry and Simulation things
    public final DifferentialDriveOdometry diffOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));

    public Odometry(Drivetrain dt) {
        drivetrain = dt;
        // Use meters as unit for encoder distances
        leftEncoder.setDistancePerPulse((2 * Math.PI * DrivetrainConstants.kWheelRadius)
                / DrivetrainConstants.kCountsPerRevolution);
        rightEncoder.setDistancePerPulse((2 * Math.PI * DrivetrainConstants.kWheelRadius)
                / DrivetrainConstants.kCountsPerRevolution);

        SmartDashboard.putData("Field", fieldSim); // This is how we can see the robot position on the field

    }


    /**
     * Update robot odometry. Odometry is the estimate of robot postion from
     * encoders and gyro
     */
    public void updateOdometry() {
        Rotation2d angle;
        if (Robot.romiConnected) {
            angle = Rotation2d.fromDegrees(romiGyro.getAngleZ());
        } else { // If we are just simulating the drivetrain
            angle = m_gyro.getRotation2d();
        }
        diffOdometry.update(
                angle, leftEncoder.getDistance(), rightEncoder.getDistance());
        fieldSim.setRobotPose(diffOdometry.getPoseMeters());
    }

    /** Resets robot odometry to a pose*/
    public void resetOdometry(Pose2d pose) {
        leftEncoder.reset();
        rightEncoder.reset();
        resetGyro();
        drivetrain.driveSim.resetSimOdometry();
        diffOdometry.resetPosition(pose, Rotation2d.fromDegrees(0));
    }

    /** Resets robot odometry. */
    public void resetOdometry() {
        resetOdometry(new Pose2d());
    }

    /** Check the current robot pose. */
    public Pose2d getPose() {
        return diffOdometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    public double getLeftDistanceInch() {
        return Conversions.metersToInches(leftEncoder.getDistance());
    }

    public double getRightDistanceInch() {
        return Conversions.metersToInches(rightEncoder.getDistance());
    }

    public double getAverageDistanceInch() {
        return Conversions.metersToInches((getLeftDistanceInch() + getRightDistanceInch()) / 2.0);
    }

    /** The acceleration in the Z-axis in Gs. */
    public double getAccelZ() {
        return accelerometer.getZ();
    }

    /** Current angle of the Romi around the Z-axis in degrees. */
    public double getGyroAngleZ() {
        return romiGyro.getAngleZ();
    }

    /** Reset the gyro. */
    public void resetGyro() {
        romiGyro.reset();
        m_gyro.reset();
    }
}
