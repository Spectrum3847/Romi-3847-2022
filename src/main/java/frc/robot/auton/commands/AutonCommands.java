package frc.robot.auton.commands;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.drivetrain.DrivetrainConstants;

/** These commands are used to build Auton Routines */
public class AutonCommands {

    /** Load the PathPlanner Trajectory from the path name, uses default MaxSpeed and MaxAccel */
    public static Trajectory getTrajectory(String pathName) {
        return getTrajectory(
                pathName, DrivetrainConstants.kMaxSpeed, DrivetrainConstants.kMaxAccel);
    }

    /** Load the PathPlanner Trajectory from the path name, maxSpeed, and maxAccel */
    public static Trajectory getTrajectory(String pathName, double maxSpeed, double maxAccel) {
        return PathPlanner.loadPath(pathName, maxSpeed, maxAccel);
    }

    /** Setup the program to run the trajectory, currently onlyd reset Odometry but may be more */
    public static Command intializePathFollowing(Trajectory path) {
        return new SequentialCommandGroup(
                AutonCommands.resetOdometry(path) // reset odometry to the initial position
                );
    }

    /** Reset the Odometry to the correct position based on the path */
    public static Command resetOdometry(Trajectory path) {
        return new InstantCommand(
                () -> Robot.drivetrain.odometry.resetOdometry(path.getInitialPose()));
    }

    /** Follow the trajectory, this is easier than typing new each time */
    public static Command followPath(Trajectory trajectory) {
        return new FollowTrajectory(trajectory);
    }

    /** Follow a path given the pathplanner name for the path */
    public static Command followPath(String pathName) {
        return followPath(getTrajectory(pathName));
    }

    /**
     * Follow the first trajectory, so it setups up odometry and gets the robot to follow more paths
     */
    public static Command followIntialPath(Trajectory trajectory) {
        return new SequentialCommandGroup(
                intializePathFollowing(trajectory), followPath(trajectory));
    }

    /** Follow the first path given the pathplanner name for the path */
    public static Command followIntialPath(String pathName) {
        return followIntialPath(getTrajectory(pathName));
    }
}
